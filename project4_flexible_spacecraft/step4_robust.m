%STEP4_ROBUST  Project 4, step 4: robust feedback for unknown panel flexibility.
%
%   The panel stiffness (EI) and damping are uncertain:
%       EI      : 0.5 - 1.5 x nominal   (panel frequency -29 % ... +22 %)
%       damping : 0.4 - 4   x nominal   (zeta = 0.2 % ... 2 %)
%   We keep the rigid PD + feedforward of step 2 and ADD a feedback controller K2
%   (same measurements: hub angle and rate errors) designed by
%     (a) H-infinity synthesis on the NOMINAL plant (hinfsyn, full order)
%     (b) mu-synthesis on the UNCERTAIN plant (musyn, D-G/K iteration, full order)
%     (c) structured robust H-infinity (systune): a 4th-order controller tuned
%         against the whole uncertainty range, constrained to be stable itself
%
%   A. Error-coordinate model: check against the step 2 simulation.
%   B. Generalised plant, weights, synthesis, controller reduction.
%   C. Robust stability (robstab), robust performance (musynperf) and the
%      closed-loop panel damping / vibration peak across the stiffness range.
%   Saves the controllers to results_step4.mat for step 4b (time-domain tests).

clear; clc; close all
here = fileparts(mfilename('fullpath'));
figdir = fullfile(here, 'figures');
p = flex_params();
J = p.J_total;  hz = @(w) w/(2*pi);
S2 = load(fullfile(here, 'results_step2.mat'));
ctrl = S2.ctrl;  theta_f = S2.theta_f;  T = S2.T_bb;
rng(4);                                            % musyn uses random starts

%% A. error-coordinate model vs the full model of step 2
[G, info] = flex_uss_err(p, 3);
Gn = G.NominalValue;
Kpd = ss([ctrl.Kp, ctrl.Kd]);  Kpd.InputName = {'y1','y2'};  Kpd.OutputName = 'upd';
% PD closed loop in error coordinates, driven by the commanded acceleration
CLe = connect(Gn, Kpd, sumblk('ufb = upd'), sumblk('y1 = -e'), sumblk('y2 = -e_dot'), ...
              ss(0,'InputName','d','OutputName','d'), 'ar', {'e','tip'});
tt = (0:0.005:100)';
[~, ~, ar] = slew_profile('bangbang', tt', theta_f, T);
ye = lsim(CLe, ar', tt, 'zoh');
o2 = S2.res(2).o;                                  % step 2: full model, both panels, ode45
e2 = interp1(o2.t, o2.theta, tt) - slew_profile('bangbang', tt', theta_f, T)';
tip2 = interp1(o2.t, o2.tip, tt);
fprintf('A. Error-coordinate model (symmetric panel modes only) vs step 2 full simulation\n');
fprintf('   bang-bang slew with PD: max |e difference| = %.1e rad (peak |e| %.1e rad)\n', ...
        max(abs(ye(:,1) - e2)), max(abs(e2)));
fprintf('                           max |tip difference| = %.1e m (peak |tip| %.1e m)\n', ...
        max(abs(ye(:,2) - tip2)), max(abs(tip2)));
fprintf('   (zoh sampling at 5 ms causes the small difference; the physics is identical)\n');

%% B. generalised plant
%   exogenous inputs : a_r (commanded acceleration, size of the bang-bang slew),
%                      d (torque disturbance 0.02 N m), sensor noise (1e-5)
%   errors to keep small: e (pointing), tip (panel vibration), u_fb (torque, rolled off)
amax = 4*theta_f / T^2;
Win = ss(diag([amax, 0.02, 1e-5, 1e-5]));
Win.InputName = {'wa','wd','n1','n2'};  Win.OutputName = {'ar','d','nn1','nn2'};
S1 = sumblk('ufb = upd + v');
S2s = sumblk('y1 = nn1 - e');  S3 = sumblk('y2 = nn2 - e_dot');
clPD = connect(Gn, Kpd, Win, S1, S2s, S3, {'wa','wd','n1','n2','v'}, {'e','tip','ufb'});
pe = norm(clPD(1,1), inf);  pt = norm(clPD(2,1), inf);
fprintf('\nB. PD alone, nominal: peak gain slew -> pointing error %.2e rad, -> tip %.2e m\n', pe, pt);
We  = ss(2/pe);  We.InputName = 'e';   We.OutputName = 'ze';     % ask for 2x less than PD
Wt  = ss(2/pt);  Wt.InputName = 'tip'; Wt.OutputName = 'zt';     % ask for 2x less than PD
Wu  = 0.25*tf([1/(2*pi*0.5) 1], [1/(2*pi*5) 1]);                 % torque: cheap below 0.5 Hz,
Wub = ss(Wu);  Wub.InputName = 'ufb';  Wub.OutputName = 'zu';     % 10x dearer above 5 Hz
P = connect(G, Kpd, Win, S1, S2s, S3, We, Wt, Wub, {'wa','wd','n1','n2','v'}, {'ze','zt','zu','y1','y2'});
fprintf('   generalised plant: %d states, uncertain parameters: %s\n', order(P.NominalValue), ...
        strjoin(fieldnames(P.Uncertainty)', ', '));

[Kh, ~, gam] = hinfsyn(P.NominalValue, 2, 1);
fprintf('   (a) H-inf, nominal plant : gamma = %.3f, controller order %d\n', gam, order(Kh));
opt = musynOptions('Display', 'off', 'MaxIter', 8, 'MixedMU', 'on');
tic;  [Km_full, ~] = musyn(P, 2, 1, opt);  tm = toc;
fprintf('   (b) mu-synthesis         : controller order %d (%.0f s)\n', order(Km_full), tm);

fprintf('       mu controller stable itself: %s (max real part of its poles %+.2f)\n', ...
        string(all(real(pole(Km_full)) < 0)), max(real(pole(Km_full))));
fprintf('       H-inf controller stable itself: %s (max real part of its poles %+.2f)\n', ...
        string(all(real(pole(Kh)) < 0)), max(real(pole(Kh))));

% (c) structured: fixed 4th-order K2, same weighted channels, tuned against the
% uncertain plant (systune alternates worst-case search and re-tuning), with the
% HARD constraint that K2's own poles are stable (decay >= 0.05 1/s).
K2t = tunableSS('K2', 4, 1, 2);  K2t.InputName = {'y1','y2'};  K2t.OutputName = 'v';
CL0 = connect(G, Kpd, K2t, Win, S1, S2s, S3, We, Wt, Wub, {'wa','wd','n1','n2'}, {'ze','zt','zu'});
soft = TuningGoal.Gain({'wa','wd','n1','n2'}, {'ze','zt','zu'}, 1);
hard = TuningGoal.ControllerPoles('K2', 0.05, 0.05, 60);
tic;  [CLt, fS, gH] = systune(CL0, soft, hard, systuneOptions('RandomStart', 3, 'Display', 'off'));
Km = ss(getBlockValue(CLt, 'K2'));
fprintf('   (c) structured robust    : order %d, worst-case weighted gain %.3f, hard %.2f (<1 met), %.0f s\n', ...
        order(Km), fS, gH, toc);
fprintf('       controller poles: %s\n', strjoin(compose('%.3f%+.3fi', [real(pole(Km)), imag(pole(Km))]), ', '));

%% C. robustness analysis
K2 = {[], Kh, Km_full, Km};
names = {'PD only', 'PD + H-inf (nominal)', 'PD + mu-synthesis', 'PD + robust (structured)'};
fprintf('\nC. Robustness over EI 0.5-1.5x, damping 0.4-4x\n');
fprintf('   %-22s  robust stab. margin   worst EI, damping    robust perf. (mu, <1 = all specs met)\n', '');
CL = cell(size(K2));
for k = 1:numel(K2)
    if isempty(K2{k}), Kk = ss(zeros(1,2)); else, Kk = K2{k}; end
    Kk.InputName = {'y1','y2'};  Kk.OutputName = 'v';
    CL{k} = connect(G, Kpd, Kk, Win, S1, S2s, S3, {'wa','wd','n1','n2'}, {'e','tip','ufb'});
    [sm, wcu] = robstab(CL{k});
    rp = musynperf(lft(P, ss(Kk)));
    if sm.LowerBound < 1.4
        wc = sprintf('x%.2f, x%.2f', wcu.kEI, wcu.gC);
    else
        wc = '(range edge)';
    end
    fprintf('   %-22s  %8.3f              %-16s   %.3f\n', names{k}, sm.LowerBound, wc, rp.UpperBound);
end
fprintf('   (margin > 1: stable for every plant in the range; 1.5 is the cap set by the\n');
fprintf('    damping parameter reaching zero 50 %% beyond its range)\n');

kg = linspace(0.5, 1.5, 41);
zeta = nan(numel(K2), numel(kg));  peak_tip = zeta;  peak_e = zeta;  maxre = zeta;
for j = 1:numel(kg)
    Gk = usubs(G, 'kEI', kg(j), 'gC', 1);
    pol = pole(Gk);  pol = pol(imag(pol) > 0);  [~, i0] = min(abs(pol));  pol = pol(i0);
    for k = 1:numel(K2)
        CLk = usubs(CL{k}, 'kEI', kg(j), 'gC', 1);
        l = pole(CLk);  maxre(k,j) = max(real(l));
        % least-damped closed-loop pole in the panel band 0.2 - 0.8 Hz
        lb = l(imag(l) > 0 & hz(abs(l)) > 0.2 & hz(abs(l)) < 0.8);
        if isempty(lb), zeta(k,j) = NaN; else, zeta(k,j) = min(-real(lb)./abs(lb)); end
        if k == 1 || k == 4, PL{k,j} = l; end %#ok<SAGROW>
        if maxre(k,j) < 0
            peak_tip(k,j) = norm(CLk(2,1), inf);  peak_e(k,j) = norm(CLk(1,1), inf);
        end
    end
end
fprintf('\n   least damping in the panel band (0.2-0.8 Hz), EI factor:   0.50   0.75   1.00   1.25   1.50\n');
pick = [1 11 21 31 41];
for k = 1:numel(K2)
    fprintf('   %-22s               ', names{k});
    for j = pick
        if maxre(k,j) >= 0, fprintf('  UNST.'); else, fprintf('%7.3f', zeta(k,j)); end
    end
    fprintf('\n');
end
fprintf('   unstable plants among the 41 EI values: %s\n', ...
        strjoin(compose('%s %d', string(names'), sum(maxre >= 0, 2)), ' | '));
fprintf('   worst-case peak tip response (over EI) / PD nominal: PD %.2f, H-inf %s, mu %.2f, structured %.2f\n', ...
        max(peak_tip(1,:))/pt, ternary(any(maxre(2,:) >= 0), 'unstable', sprintf('%.2f', max(peak_tip(2,:))/pt)), ...
        max(peak_tip(3,:))/pt, max(peak_tip(4,:))/pt);
fprintf('   worst-case peak pointing response / PD nominal: PD %.2f, mu %.2f, structured %.2f\n', ...
        max(peak_e(1,:))/pe, max(peak_e(3,:))/pe, max(peak_e(4,:))/pe);

save(fullfile(here, 'results_step4.mat'), 'Kh', 'Km', 'Km_full', 'kg', 'zeta', 'peak_tip', ...
     'peak_e', 'maxre', 'names', 'pt', 'pe');

%% figures
f = figure('Position', [80 80 1400 430]); theme(f, 'light');
cols = [0 0 0; lines(3)];
subplot(1,3,1); hold on; grid on
for k = [1 2 4]
    z = zeta(k,:);  z(maxre(k,:) >= 0) = NaN;
    plot(kg, z, 'Color', cols(k,:), 'LineWidth', 1.8);
    if any(maxre(k,:) >= 0)
        plot(kg(maxre(k,:) >= 0), zeros(1, sum(maxre(k,:) >= 0)), 'x', 'Color', cols(k,:), ...
             'MarkerSize', 7, 'LineWidth', 1.5, 'HandleVisibility', 'off');
    end
end
xline(1, 'k:'); xlabel('true panel stiffness / nominal'); ylabel('least damping ratio, 0.2-0.8 Hz');
title('Closed-loop panel damping (x = unstable)');
legend(names([1 2 4]), 'Location', 'west');
subplot(1,3,2); hold on; grid on
for k = [1 2 3 4]
    plot(kg, peak_tip(k,:)/pt, 'Color', cols(k,:), 'LineWidth', 1.8);
end
set(gca, 'YScale', 'log'); xline(1, 'k:');
xlabel('true panel stiffness / nominal'); ylabel('peak gain slew \rightarrow tip  (PD nominal = 1)');
title('Worst-frequency panel response');
legend(names, 'Location', 'northeast');
subplot(1,3,3); hold on; grid on
% closed-loop poles near the panel mode as the stiffness varies
cm = parula(numel(kg));
for j = 1:numel(kg)
    for k = [1 4]
        l = PL{k,j};  l = l(imag(l) > 0 & hz(abs(l)) > 0.2 & hz(abs(l)) < 0.8);
        if k == 1, mk = 'o'; else, mk = 's'; end
        plot(real(l), hz(imag(l)), mk, 'Color', cm(j,:), 'MarkerFaceColor', cm(j,:)*(k==4) + [1 1 1]*(k==1), ...
             'MarkerSize', 4, 'HandleVisibility', 'off');
    end
end
plot(nan, nan, 'ko', 'DisplayName', 'PD only');
plot(nan, nan, 'ks', 'MarkerFaceColor', 'k', 'DisplayName', 'PD + robust (structured)');
th = linspace(0, pi/2, 50);
zls = [0.05 0.1 0.2 0.3];  ytx = [0.52 0.62 0.70 0.77];
for q = 1:numel(zls)
    zl = zls(q);  wl = linspace(0, 2*pi*0.8, 50);
    plot(-zl/sqrt(1 - zl^2)*wl, hz(wl), 'k:', 'HandleVisibility', 'off');
    text(-zl/sqrt(1 - zl^2)*2*pi*ytx(q) - 0.02, ytx(q), sprintf('\\zeta=%.2f', zl), 'FontSize', 8, ...
         'HorizontalAlignment', 'right');
end
xlim([-1.6 0.05]); ylim([0.2 0.8]);
colormap(gca, parula); cb = colorbar; clim([0.5 1.5]); cb.Label.String = 'panel stiffness / nominal';
xlabel('real part [1/s]'); ylabel('frequency [Hz]');
title('Closed-loop poles in the panel band');
legend('Location', 'southwest');
exportgraphics(f, fullfile(figdir, 'step4_robust.png'), 'Resolution', 150);
fprintf('\nsaved %s\n', fullfile(figdir, 'step4_robust.png'));

function s = ternary(c, a, b)
if c, s = a; else, s = b; end
end
