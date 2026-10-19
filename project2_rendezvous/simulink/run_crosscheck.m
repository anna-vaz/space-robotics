%RUN_CROSSCHECK  Build the Simulink attitude model, simulate it, and compare
%   with the Python simulation (project 2, step 3 part B).
%   Run from this folder:  matlab -batch "run_crosscheck"
%   (first run  python export_for_simulink.py  to create crosscheck_data.mat)

clear; close all;
here = fileparts(mfilename('fullpath'));
if isempty(here), here = pwd; end
cd(here);
load('crosscheck_data.mat');            % J K P U_MAX n beta0 omega0 t beta_py omega_py u_py
beta0 = beta0(:); omega0 = omega0(:);

model = 'attitude_control';
if bdIsLoaded(model), close_system(model, 0); end
slx = fullfile(here, [model '.slx']);
if isfile(slx), delete(slx); end         % rebuild from scratch every time
build_attitude_model(model);
save_system(model, fullfile(here, [model '.slx']));
print(['-s' model], '-dpng', '-r150', fullfile(here, 'figures', 'simulink_model.png'));

tic; out = sim(model); t_run = toc;
t_sl = out.t_sl(:);
beta_sl = asRows(out.beta_sl, 4); omega_sl = asRows(out.omega_sl, 3); u_sl = asRows(out.u_sl, 3);
fprintf('logged sizes: beta %s, omega %s, u %s\n', mat2str(size(beta_sl)), mat2str(size(omega_sl)), mat2str(size(u_sl)));
t_py = t(:);
assert(numel(t_sl) == numel(t_py) && max(abs(t_sl - t_py)) < 1e-9, 'time grids differ');

% ---- comparison ------------------------------------------------------------
d_beta = max(abs(beta_sl - beta_py), [], 'all');
d_omega = max(abs(omega_sl - omega_py), [], 'all');
d_u = max(abs(u_sl - u_py), [], 'all');
ang = zeros(numel(t_py), 1); err_sl = ang; err_py = ang;
for k = 1:numel(t_py)
    ang(k) = relAngle(beta_sl(k,:), beta_py(k,:));            % Simulink vs Python attitude
    RN = refRN(n, t_py(k));
    err_sl(k) = relAngleDCM(ep2dcm(beta_sl(k,:)), RN);
    err_py(k) = relAngleDCM(ep2dcm(beta_py(k,:)), RN);
end
settle = @(e) t_py(find(rad2deg(e) > 0.1, 1, 'last') + 1);
fprintf('Simulink run time: %.1f s (%d output samples)\n', t_run, numel(t_sl));
fprintf('max |beta_SL - beta_PY|   = %.2e\n', d_beta);
fprintf('max |omega_SL - omega_PY| = %.2e rad/s\n', d_omega);
fprintf('max |u_SL - u_PY|         = %.2e N m\n', d_u);
fprintf('max attitude difference   = %.2e deg\n', rad2deg(max(ang)));
fprintf('pointing error < 0.1 deg after: Simulink %.1f s, Python %.1f s\n', settle(err_sl), settle(err_py));
fprintf('final pointing error: Simulink %.2e deg, Python %.2e deg\n', rad2deg(err_sl(end)), rad2deg(err_py(end)));
fprintf('|beta| drift (Simulink): %.1e\n', max(abs(vecnorm(beta_sl, 2, 2) - 1)));

% ---- figure ---------------------------------------------------------------
f = figure('Visible', 'off', 'Position', [100 100 1200 750]);
theme(f, 'light');
tiledlayout(2, 2, 'Padding', 'compact');
nexttile; hold on; grid on;
plot(t_py, beta_sl, 'LineWidth', 1.5);
set(gca, 'ColorOrderIndex', 1);
plot(t_py(1:50:end), beta_py(1:50:end, :), 'o', 'MarkerSize', 4);
title('Euler parameters: Simulink (lines) vs Python (circles)');
xlabel('time [s]'); legend('\beta_0', '\beta_1', '\beta_2', '\beta_3', 'Location', 'north', 'NumColumns', 4);
nexttile; semilogy(t_py, rad2deg(err_sl) + 1e-12, 'LineWidth', 1.5); hold on; grid on;
semilogy(t_py(1:50:end), rad2deg(err_py(1:50:end)) + 1e-12, 'o', 'MarkerSize', 4);
yline(0.1, ':'); title('Pointing error [deg]'); xlabel('time [s]'); legend('Simulink', 'Python', '0.1 deg');
nexttile; plot(t_py, u_sl, 'LineWidth', 1.5); grid on; xlim([0 400]); ylim(1.3*U_MAX*[-1 1]);
yline(U_MAX, ':'); yline(-U_MAX, ':');
title('Control torque, Simulink [N m]'); xlabel('time [s]'); legend('u_1', 'u_2', 'u_3');
nexttile; semilogy(t_py, max(abs(beta_sl - beta_py), [], 2) + 1e-18, 'LineWidth', 1.2); hold on; grid on;
semilogy(t_py, rad2deg(ang) + 1e-18, 'LineWidth', 1.2);
title('Simulink - Python difference'); xlabel('time [s]');
legend('max |\Delta\beta|', 'attitude difference [deg]', 'Location', 'best');
exportgraphics(f, fullfile(here, 'figures', 'simulink_vs_python.png'), 'Resolution', 150);
fprintf('saved figures to %s\n', fullfile(here, 'figures'));

% ---- local helper functions -------------------------------------------------
function A = asRows(A, m)
% To Workspace logs column-vector signals as m x 1 x N; return N x m
A = squeeze(A);
if size(A, 1) == m && size(A, 2) ~= m, A = A.'; end
end

function C = ep2dcm(b)
C = [b(1)^2+b(2)^2-b(3)^2-b(4)^2, 2*(b(2)*b(3)+b(1)*b(4)), 2*(b(2)*b(4)-b(1)*b(3));
     2*(b(2)*b(3)-b(1)*b(4)), b(1)^2-b(2)^2+b(3)^2-b(4)^2, 2*(b(3)*b(4)+b(1)*b(2));
     2*(b(2)*b(4)+b(1)*b(3)), 2*(b(3)*b(4)-b(1)*b(2)), b(1)^2-b(2)^2-b(3)^2+b(4)^2];
end

function RN = refRN(n, t)
r1 = [-sin(n*t); cos(n*t); 0]; r3 = [0; 0; 1];
RN = [r1.'; cross(r3, r1).'; r3.'];
end

function a = relAngleDCM(A, B)
% principal rotation angle between two attitudes, accurate near zero
C = A*B.';
v = [C(2,3)-C(3,2); C(3,1)-C(1,3); C(1,2)-C(2,1)];
a = atan2(norm(v)/2, (trace(C)-1)/2);
end

function a = relAngle(b1, b2)
a = relAngleDCM(ep2dcm(b1), ep2dcm(b2));
end
