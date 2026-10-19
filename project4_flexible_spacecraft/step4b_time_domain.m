%STEP4B_TIME_DOMAIN  Project 4, step 4b: the controllers on real slews.
%
%   Full model (both panels, 3 modes each, torque limit 1 N m), the 90-degree
%   bang-bang slew of step 2, four strategies:
%       PD                 rigid design (step 2)
%       PD + ZVD shaper    tuned to the nominal panels (step 3)
%       PD + H-inf         designed for the nominal panels (step 4)
%       PD + robust        structured robust H-inf, whole uncertainty range (step 4)
%   A. Sweep of the panel stiffness 0.5-1.5x (nominal damping).
%   B. Monte Carlo: 300 spacecraft with random stiffness AND damping.

clear; clc; close all
here = fileparts(mfilename('fullpath'));
figdir = fullfile(here, 'figures');
p = flex_params();
J = p.J_total;  arcsec = 180/pi*3600;
S2 = load(fullfile(here, 'results_step2.mat'));
S3 = load(fullfile(here, 'results_step3.mat'));
S4 = load(fullfile(here, 'results_step4.mat'));
ctrl0 = S2.ctrl;  theta_f = S2.theta_f;  T = S2.T_bb;
[Az, tz] = input_shaper('ZVD', S3.f_d, S3.z_d);

strat = {'PD', 'PD + ZVD shaper', 'PD + H-inf (nominal)', 'PD + robust (structured)'};
mk_ctrl = @(K2) setfield(ctrl0, 'K2', K2); %#ok<SFLD>
C = {mk_ctrl([]), mk_ctrl([]), mk_ctrl(S4.Kh), mk_ctrl(S4.Km)};
shp = {{1, 0}, {Az, tz}, {1, 0}, {1, 0}};
t_end = 160;

run1 = @(pk, k) one_slew(pk, C{k}, shp{k}, theta_f, T, J, t_end);

%% A. stiffness sweep
EIs = 0.5:0.1:1.5;
ts = nan(numel(strat), numel(EIs));  res_tip = ts;  peak_u = ts;
keep = struct();
for j = 1:numel(EIs)
    pj = p;  pj.EI = p.EI*EIs(j);
    for k = 1:numel(strat)
        o = run1(pj, k);
        ts(k,j) = o.ts;  res_tip(k,j) = o.res_tip;  peak_u(k,j) = o.peak_u;
        if any(abs(EIs(j) - [0.6 1.0]) < 1e-9), keep(k, round(EIs(j)*10)).o = o; end
    end
end
fprintf('A. Settling time [s] (0.001 deg and 1 mm) of a 90-degree slew; slew command lasts %.1f s\n', T);
fprintf('   EI factor:            '); fprintf('%7.1f', EIs); fprintf('\n');
for k = 1:numel(strat)
    fprintf('   %-22s', strat{k});
    for j = 1:numel(EIs)
        if isnan(ts(k,j)), fprintf('  FAIL '); else, fprintf('%7.1f', ts(k,j)); end
    end
    fprintf('\n');
end
fprintf('   peak torque over the sweep [N m]: %s\n', strjoin(compose('%s %.2f', string(strat'), max(peak_u, [], 2)), ' | '));

%% B. Monte Carlo over stiffness and damping
N = 300;
rng(44);
kEI = 0.5 + rand(N,1);                              % uniform 0.5 - 1.5
gC  = exp(log(0.4) + rand(N,1)*log(10));            % log-uniform 0.4 - 4
TS = nan(N, numel(strat));  RT = TS;
fprintf('\nB. Monte Carlo, %d random spacecraft ...\n', N);
tic
parfor i = 1:N
    pi_ = p;  pi_.EI = p.EI*kEI(i);  pi_.zeta = p.zeta*gC(i);
    row = nan(1, numel(strat));  rrow = row;
    for k = 1:numel(strat)
        o = one_slew(pi_, C{k}, shp{k}, theta_f, T, J, t_end);
        row(k) = o.ts;  rrow(k) = o.res_tip;
    end
    TS(i,:) = row;  RT(i,:) = rrow;
end
fprintf('   done in %.0f s\n', toc);
fprintf('   %-22s  failed     settled <60 s   median   95th pct   worst [s]\n', '');
for k = 1:numel(strat)
    s = TS(:,k);  st = s(~isnan(s));
    fprintf('   %-22s  %5d     %5d / %d    %6.1f   %7.1f   %7.1f\n', strat{k}, sum(isnan(s)), ...
            sum(st < 60), N, median(st), prctile(st, 95), max(st));
end
%% C. why does the nominal H-inf design fail even on the nominal panels?
pn = p;  m = flex_model(pn);  c3 = C{3};
ref = @(t) shaped(t, 1, 0, theta_f, T, J);
fprintf('\nC. PD + H-inf on the NOMINAL panels\n');
for umax = [1 1e6]
    pn.u_max = umax;
    o = flex_simulate(m, pn, c3, ref, 120, [T/2 T]);
    fprintf('   torque limit %-7g: final pointing error %10.3g deg, peak torque %.2f N m, saturated %3.0f %% of the time\n', ...
            umax, (o.theta(end) - theta_f)*180/pi, max(abs(o.u)), 100*mean(abs(o.u) >= 1 - 1e-9));
end
ph = pole(S4.Kh);  ph = ph(real(ph) > 0);
fprintf('   H-inf controller poles with positive real part: %s\n', ...
        strjoin(compose('%.3f%+.3fi', [real(ph), imag(ph)]), ', '));
fprintf('   -> linear closed loop is stable, but the controller itself is unstable: once the\n');
fprintf('      torque saturates, its unstable state runs away and the spacecraft tumbles.\n');

save(fullfile(here, 'results_step4b.mat'), 'EIs', 'ts', 'res_tip', 'kEI', 'gC', 'TS', 'RT', 'strat');

%% figures
f = figure('Position', [60 60 1450 820]); theme(f, 'light');
cols = [0 0 0; lines(3)];
for c = 1:2
    col = [6 10];  ttl = {'panels 40 % softer than designed for (EI x 0.6)', 'nominal panels (EI x 1.0)'};
    subplot(2,3,c); hold on; grid on
    for k = 1:numel(strat)
        o = keep(k, col(c)).o;
        if isnan(o.ts), continue; end
        plot(o.t, 1e3*o.tip, 'Color', cols(k,:), 'LineWidth', 1.0);
    end
    xlim([0 100]); ylim([-8 8]); xlabel('time [s]'); ylabel('tip deflection [mm]'); title(ttl{c});
    if c == 1, legend(strat(~isnan(arrayfun(@(k) keep(k,6).o.ts, 1:4))), 'Location', 'southeast'); end
end
subplot(2,3,3); hold on; grid on
for k = 1:numel(strat)
    y = ts(k,:);
    plot(EIs, y, '-o', 'Color', cols(k,:), 'LineWidth', 1.6, 'MarkerSize', 4, 'MarkerFaceColor', cols(k,:));
    bad = isnan(y);
    if any(bad), plot(EIs(bad), 40*ones(1,sum(bad)), 'x', 'Color', cols(k,:), 'MarkerSize', 9, 'LineWidth', 2, 'HandleVisibility', 'off'); end
end
xline(1, 'k:'); xlabel('true panel stiffness / nominal'); ylabel('settling time [s]');
title('Settling time (x on the axis = failed)'); legend(strat, 'Location', 'northeast');
subplot(2,3,4); hold on; grid on
for c = 1:2
    for k = 1:numel(strat)
        o = keep(k, col(c)).o;
        if isnan(o.ts) || c == 2, continue; end
        w = o.t >= T;
        plot(o.t(w), arcsec*abs(o.theta(w) - theta_f), 'Color', cols(k,:), 'LineWidth', 1.0);
    end
end
set(gca, 'YScale', 'log'); ylim([1e-2 1e3]); xlim([T 110]);
yline(3.6, 'k--', '0.001^\circ', 'LabelHorizontalAlignment', 'left');
xlabel('time [s]'); ylabel('|pointing error| [arcsec]'); title('Pointing error after the slew (EI x 0.6)');
subplot(2,3,5); hold on; grid on
for k = 1:numel(strat)
    s = sort(TS(:,k));  s(isnan(s)) = Inf;
    stairs(s, (1:N)'/N*100, 'Color', cols(k,:), 'LineWidth', 1.8);
end
xlim([40 160]); xlabel('settling time [s]'); ylabel('% of spacecraft settled');
title(sprintf('Monte Carlo: %d random panels (stiffness and damping)', N));
legend(strat, 'Location', 'southeast');
subplot(2,3,6); hold on; grid on
k = 4;  scatter(kEI, TS(:,2), 14, cols(2,:), 'filled');  scatter(kEI, TS(:,k), 14, cols(k,:), 'filled');
xlabel('panel stiffness / nominal'); ylabel('settling time [s]');
title('Monte Carlo: shaper vs robust feedback'); legend(strat([2 4]), 'Location', 'north');
exportgraphics(f, fullfile(figdir, 'step4b_time_domain.png'), 'Resolution', 150);
fprintf('\nsaved %s\n', fullfile(figdir, 'step4b_time_domain.png'));

%% local
function out = one_slew(p, ctrl, shp, theta_f, T, J, t_end)
m = flex_model(p);
A = shp{1};  tk = shp{2};
ref = @(t) shaped(t, A, tk, theta_f, T, J);
br = unique([tk, tk + T/2, tk + T]);
o = flex_simulate(m, p, ctrl, ref, t_end, br);
err = o.theta - theta_f;
if ~all(isfinite(o.tip)) || max(abs(o.tip)) > 0.5 || abs(err(end)) > 1e-2
    out = struct('ts', NaN, 'res_tip', NaN, 'peak_u', NaN, 't', o.t, 'tip', o.tip, 'theta', o.theta);
    return
end
bad = abs(err) > 1e-3*pi/180 | abs(o.tip) > 1e-3;
ts = o.t(find(bad, 1, 'last') + 1);
if isempty(ts) || ts >= t_end - 1, ts = NaN; end
Te = T + tk(end);  w = o.t >= Te & o.t <= Te + 20;
out = struct('ts', ts, 'res_tip', max(abs(o.tip(w))), 'peak_u', max(abs(o.u)), ...
             't', o.t, 'tip', o.tip, 'theta', o.theta);
end

function r = shaped(t, A, tk, theta_f, T, J)
r = zeros(3, numel(t));
for i = 1:numel(A)
    [th, thd, thdd] = slew_profile('bangbang', t - tk(i), theta_f, T);
    r = r + A(i)*[th; thd; J*thdd];
end
end
