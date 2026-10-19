%STEP3_INPUT_SHAPING  Project 4, step 3: input shaping of the slew command.
%
%   Same spacecraft, same rigid PD + feedforward as step 2. The bang-bang command
%   is convolved with a short impulse sequence (ZV, ZVD, EI) tuned to the panel
%   mode, so that the vibration started by one impulse is cancelled by the next.
%
%   A. Nominal plant: time histories, residual vibration, settling time.
%   B. Robustness: the panel stiffness EI is NOT known exactly. Sweep the true EI
%      from 0.5x to 1.5x the design value; the shaper stays tuned to the nominal.
%      Compare the simulated residual with the shaper's sensitivity formula.

clear; clc; close all
here = fileparts(mfilename('fullpath'));
figdir = fullfile(here, 'figures');
p = flex_params();
m = flex_model(p);
J = p.J_total;  ndof = size(m.M,1);
hz = @(w) w/(2*pi);  arcsec = 180/pi*3600;
S2 = load(fullfile(here, 'results_step2.mat'));       % gains, slew, durations of step 2
ctrl = S2.ctrl;  theta_f = S2.theta_f;  T = S2.T_bb;

% panel mode of the NOMINAL closed loop: this is what the shapers are tuned to
lam = cl_panel_pole(m, ctrl, ndof);
f_d = hz(imag(lam));  z_d = -real(lam)/abs(lam);
fprintf('Design mode (closed loop, nominal panels): f_d = %.4f Hz, zeta = %.4f, period %.3f s\n', ...
        f_d, z_d, 1/f_d);

shapers = {'none', 'ZV', 'ZVD', 'EI'};
for k = 1:numel(shapers)
    [A, tk] = input_shaper(shapers{k}, f_d, z_d);
    fprintf('   %-4s impulses A = [%s], at t = [%s] s (adds %.2f s)\n', shapers{k}, ...
            strjoin(compose('%.4f', A), ', '), strjoin(compose('%.3f', tk), ', '), tk(end));
end

%% A. nominal plant, full nonlinear simulation (torque limit)
fprintf('\nA. 90-degree bang-bang slew, nominal panels\n');
fprintf('   shaper   slew ends   residual tip   pointing after slew   settled (0.001 deg & 1 mm)\n');
tol_th = 1e-3*pi/180;  tol_tip = 1e-3;
res = struct();
for k = 1:numel(shapers)
    [A, tk] = input_shaper(shapers{k}, f_d, z_d);
    ref = @(t) shaped_ref(t, A, tk, theta_f, T, J);
    br  = unique([tk + T/2, tk + T, tk]);
    o   = flex_simulate(m, p, ctrl, ref, 150, br);
    Te  = T + tk(end);
    w   = o.t >= Te & o.t <= Te + 20;
    err = o.theta - theta_f;
    bad = abs(err) > tol_th | abs(o.tip) > tol_tip;
    ts  = o.t(find(bad, 1, 'last') + 1);
    fprintf('   %-5s   %7.2f s   %9.3f mm     %8.2f arcsec      %7.1f s\n', shapers{k}, Te, ...
            1e3*max(abs(o.tip(w))), arcsec*max(abs(err(w))), ts);
    res(k).o = o;  res(k).Te = Te;  res(k).ts = ts;  res(k).name = shapers{k};
    res(k).peak_u = max(abs(o.u));
end
fprintf('   peak torque: %s N m\n', strjoin(compose('%.3f', [res.peak_u]), ', '));

%% B. robustness to the panel stiffness
EIf = linspace(0.5, 1.5, 101);
Rsim = zeros(numel(shapers), numel(EIf));  Rhand = Rsim;  fact = zeros(size(EIf));
dt = 0.005;
for j = 1:numel(EIf)
    pj = p;  pj.EI = p.EI * EIf(j);
    mj = flex_model(pj);
    lj = cl_panel_pole(mj, ctrl, ndof);   fact(j) = hz(imag(lj));
    for k = 1:numel(shapers)
        [A, tk] = input_shaper(shapers{k}, f_d, z_d);
        Te = T + tk(end);
        tt = (0:dt:Te + 6)';
        r  = shaped_ref(tt', A, tk, theta_f, T, J);
        y  = lsim_cl(mj, ctrl, r', tt, ndof);
        Rsim(k, j) = max(abs(y(tt >= Te)));
        % hand formula: the shaped command is a sum of delayed copies, so its
        % vibration at the end (t = T + t_n) is the unshaped one times
        %     V = | sum_i A_i e^{lambda (t_n - t_i)} |   (lambda = TRUE panel pole);
        % without damping and at the design frequency this is the shaper's
        % classical "vibration ratio", 0 for ZV/ZVD.
        V = abs(sum(A .* exp(lj * (tk(end) - tk))));
        Rhand(k, j) = V;            % scaled below by the unshaped residual
    end
end
Rhand = Rhand .* Rsim(1, :);        % shaped residual = V * unshaped residual
fprintf('\nB. Panel stiffness 0.5-1.5 x nominal (panel frequency %.3f-%.3f Hz)\n', fact(1), fact(end));
fprintf('   residual tip vibration [mm] for selected stiffness errors:\n');
pick = [0.5 0.7 0.8 0.9 1.0 1.1 1.2 1.3 1.5];
fprintf('   EI factor:'); fprintf('%8.2f', pick); fprintf('\n');
for k = 1:numel(shapers)
    fprintf('   %-9s ', shapers{k});
    fprintf('%8.3f', 1e3*interp1(EIf, Rsim(k,:), pick));
    fprintf('\n');
end
dif = abs(Rhand(2:end,:) - Rsim(2:end,:));
fprintf('   sensitivity formula vs simulation (shaped cases): max |difference| = %.3f mm, median %.3f mm\n', ...
        1e3*max(dif(:)), 1e3*median(dif(:)));
[~, im] = max(dif(:));  [ks, js] = ind2sub(size(dif), im);
fprintf('   largest difference: %s at EI x %.2f, where the residual is %.2f mm (%.0f %% error)\n', ...
        shapers{ks+1}, EIf(js), 1e3*Rsim(ks+1, js), 100*dif(im)/Rsim(ks+1, js));
fprintf('   (the formula keeps only the first panel mode and its value at the end of the slew;\n');
fprintf('    the simulation takes the largest deflection in the next 6 s, all modes)\n');
lim = 0.5e-3;                                    % "good enough": below 0.5 mm
fprintf('   stiffness range with residual < %.1f mm:\n', 1e3*lim);
for k = 1:numel(shapers)
    ok = Rsim(k,:) < lim;
    if any(ok)
        % contiguous band around the nominal (EI factor 1)
        i0 = find(abs(EIf - 1) < 1e-9);
        if ok(i0)
            a = i0; while a > 1 && ok(a-1), a = a - 1; end
            b = i0; while b < numel(EIf) && ok(b+1), b = b + 1; end
            fprintf('     %-5s EI x %.2f - %.2f  (panel frequency %+.1f %% to %+.1f %%)\n', shapers{k}, ...
                    EIf(a), EIf(b), 100*(fact(a)/f_d - 1), 100*(fact(b)/f_d - 1));
        else
            fprintf('     %-5s not even at the nominal stiffness\n', shapers{k});
        end
    else
        fprintf('     %-5s nowhere\n', shapers{k});
    end
end
save(fullfile(here, 'results_step3.mat'), 'EIf', 'Rsim', 'shapers', 'f_d', 'z_d');

%% figures
f = figure('Position', [80 80 1400 800]); theme(f, 'light');
cols = lines(4);
subplot(2,2,1); hold on; grid on
for k = 1:numel(shapers), plot(res(k).o.t, res(k).o.u, 'Color', cols(k,:), 'LineWidth', 1.2); end
xlim([15 50]); xlabel('time [s]'); ylabel('torque [N m]');
title('Torque around the switch and the end of the slew'); legend(shapers, 'Location', 'northeast');
subplot(2,2,2); hold on; grid on
for k = 1:numel(shapers), plot(res(k).o.t, 1e3*res(k).o.tip, 'Color', cols(k,:), 'LineWidth', 1.0); end
xlim([0 80]); xlabel('time [s]'); ylabel('tip deflection [mm]');
title('Panel tip deflection (nominal panels)'); legend(shapers, 'Location', 'northeast');
subplot(2,2,3); hold on; grid on
for k = 1:numel(shapers)
    o = res(k).o;  w = o.t >= res(k).Te - 0.01;
    plot(o.t(w), arcsec*abs(o.theta(w) - theta_f), 'Color', cols(k,:), 'LineWidth', 1.0);
end
set(gca, 'YScale', 'log'); ylim([1e-3 1e2]); xlim([38 100]);
yline(3.6, 'k--', '0.001^\circ', 'LabelHorizontalAlignment', 'left');
xlabel('time [s]'); ylabel('|pointing error| [arcsec]'); title('Pointing error after the slew');
legend(shapers, 'Location', 'northeast');
subplot(2,2,4); hold on; grid on
for k = 1:numel(shapers)
    plot(EIf, 1e3*Rsim(k,:), 'Color', cols(k,:), 'LineWidth', 1.8);
end
for k = 2:numel(shapers), plot(EIf, 1e3*Rhand(k,:), 'k:', 'LineWidth', 1.0, 'HandleVisibility', 'off'); end
yline(1e3*lim, 'k--', '0.5 mm', 'HandleVisibility', 'off');
xline(1, 'k-', 'design', 'HandleVisibility', 'off', 'LabelVerticalAlignment', 'bottom');
set(gca, 'YScale', 'log'); ylim([5e-3 10]);
xlabel('true panel stiffness / design value'); ylabel('residual tip vibration [mm]');
title('Robustness to the panel stiffness (dotted: sensitivity formula)');
legend(shapers, 'Location', 'southwest');
exportgraphics(f, fullfile(figdir, 'step3_input_shaping.png'), 'Resolution', 150);
fprintf('\nsaved %s\n', fullfile(figdir, 'step3_input_shaping.png'));

%% local functions
function r = shaped_ref(t, A, tk, theta_f, T, J)
% reference [theta; theta_dot; u_ff] = sum_i A_i * (bang-bang command delayed by t_i)
r = zeros(3, numel(t));
for i = 1:numel(A)
    [th, thd, thdd] = slew_profile('bangbang', t - tk(i), theta_f, T);
    r = r + A(i) * [th; thd; J*thdd];
end
end

function lam = cl_panel_pole(m, ctrl, ndof)
% closed-loop pole of the coupled panel mode (nearest to the free coupled mode)
K = [ctrl.Kp, zeros(1,ndof-1), ctrl.Kd, zeros(1,ndof-1)];
l = eig(m.sys.A - m.sys.B*K);
[V, D] = eig(m.Kq, m.M);  w = sqrt(abs(diag(D)));
d = abs(m.d(1));
% the coupled first mode is the free mode whose shape moves the hub
[~, o] = sort(w);  w = w(o);  V = V(:, o);
wc = w(find(abs(V(1, :)) > 1e-8 & w' > 1e-6, 1));
l = l(imag(l) > 0);
[~, j] = min(abs(l - 1i*wc));  lam = l(j);
end

function tip = lsim_cl(m, ctrl, r, t, ndof)
A = m.sys.A;  B = m.sys.B;
K = [ctrl.Kp, zeros(1,ndof-1), ctrl.Kd, zeros(1,ndof-1)];
tip = lsim(ss(A - B*K, B*[ctrl.Kp, ctrl.Kd, 1], m.sys.C(3,:), 0), r, t, 'foh');
end
