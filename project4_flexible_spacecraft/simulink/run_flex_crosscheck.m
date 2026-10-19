%RUN_FLEX_CROSSCHECK  Simulink version of the robust slew vs the MATLAB simulation.
%
%   Builds flex_slew.slx (build_flex_simulink.m), runs the 90-degree bang-bang slew
%   with PD + structured robust controller on the nominal spacecraft, and compares
%   it sample by sample with flex_simulate.m (ode45 with restarts at the switches).
here = fileparts(mfilename('fullpath'));
root = fileparts(here);
addpath(root);
figdir = fullfile(here, 'figures');  if ~exist(figdir, 'dir'), mkdir(figdir); end
p = flex_params();  m = flex_model(p);  J = p.J_total;
S2 = load(fullfile(root, 'results_step2.mat'));
S4 = load(fullfile(root, 'results_step4.mat'));
ctrl = S2.ctrl;  ctrl.K2 = S4.Km;  theta_f = S2.theta_f;  T = S2.T_bb;

% MATLAB reference run
ref = @(t) refb(t, theta_f, T, J);
tic;  o = flex_simulate(m, p, ctrl, ref, 120, [T/2 T]);  t_ml = toc;

% Simulink
[Ak, Bk, Ck, Dk] = ssdata(S4.Km);
P = struct('A', m.sys.A, 'B', m.sys.B, 'C', m.sys.C, 'D', m.sys.D, 'Ak', Ak, 'Bk', Bk, 'Ck', Ck, ...
           'Dk', Dk, 'Kp', ctrl.Kp, 'Kd', ctrl.Kd, 'J', J, 'a', 4*theta_f/T^2, 'T', T, ...
           'umax', p.u_max, 't_out', o.t);
mdl = build_flex_simulink(here, P);
tic;  out = sim(mdl);  t_sl = toc;
y = out.y_sl;  u = out.u_sl;
if ndims(y) == 3, y = squeeze(y)'; end
fprintf('Simulink vs MATLAB, 90-degree slew, PD + structured robust controller, nominal panels\n');
fprintf('   samples compared: %d (Simulink %d)\n', numel(o.t), size(y, 1));
fprintf('   max |theta difference|  = %.2e rad   (%.2e arcsec)\n', max(abs(y(:,1) - o.theta)), ...
        max(abs(y(:,1) - o.theta))*180/pi*3600);
fprintf('   max |rate difference|   = %.2e rad/s\n', max(abs(y(:,2) - o.theta_dot)));
fprintf('   max |tip difference|    = %.2e m     (peak tip %.2e m)\n', max(abs(y(:,3) - o.tip)), max(abs(o.tip)));
sw = abs(o.t - T/2) < 1e-9 | abs(o.t - T) < 1e-9;      % the two switching instants
fprintf('   max |torque difference| = %.2e N m (excluding the 2 samples exactly at the switches,\n', ...
        max(abs(u(~sw) - o.u(~sw))));
fprintf('                             where one side reports the torque before and the other after)\n');
fprintf('   run time: MATLAB %.1f s, Simulink %.1f s\n', t_ml, t_sl);

f = figure('Position', [100 100 1200 380]); theme(f, 'light');
subplot(1,3,1); plot(o.t, o.theta*180/pi, 'LineWidth', 2); hold on; grid on
plot(o.t, y(:,1)*180/pi, '--', 'LineWidth', 1.5); xlabel('time [s]'); ylabel('hub angle [deg]');
legend('MATLAB (ode45)', 'Simulink', 'Location', 'southeast'); title('Slew of 90^\circ');
subplot(1,3,2); plot(o.t, 1e3*o.tip, 'LineWidth', 2); hold on; grid on
plot(o.t, 1e3*y(:,3), '--', 'LineWidth', 1.5); xlabel('time [s]'); ylabel('tip deflection [mm]');
title('Panel tip'); xlim([0 80]);
subplot(1,3,3); semilogy(o.t, abs(y(:,1) - o.theta) + 1e-18, 'LineWidth', 1.2); hold on; grid on
semilogy(o.t, abs(y(:,3) - o.tip) + 1e-18, 'LineWidth', 1.2);
xlabel('time [s]'); ylabel('|Simulink - MATLAB|'); legend('hub angle [rad]', 'tip [m]', 'Location', 'southeast');
title('Difference'); ylim([1e-16 1e-6]);
exportgraphics(f, fullfile(figdir, 'simulink_vs_matlab.png'), 'Resolution', 150);
open_system(mdl);
print(['-s' mdl], '-dpng', '-r150', fullfile(figdir, 'simulink_model.png'));
close_system(mdl, 0);
fprintf('saved figures in %s\n', figdir);

function r = refb(t, th, T, J)
[a, b, c] = slew_profile('bangbang', t, th, T);
r = [a; b; J*c];
end
