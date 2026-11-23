%RUN_RNS_SIMSCAPE  Project 1b on a Simscape Multibody plant: paths A and B, three controllers.
%
%   Same robot (params.py), start configuration, paths and gains as step3_path_tracking.py.
%   Saves rns_simscape_results.mat for compare_rns_simscape.py.
%
%   Run from this folder:   matlab -batch "run_rns_simscape"

clear; clc;
here = fileparts(mfilename('fullpath'));
cd(here);

% ---- robot (params.py) ------------------------------------------------------------
m0 = 100;  a0 = 1;  I0 = m0*(a0^2 + a0^2)/12;  b0 = [a0/2 0];
LEN = [1 1 0.5];  MS = [10 10 5];  IN = MS.*LEN.^2/12;

% ---- task and controller (step3_path_tracking.py / path_tracking.py) ---------------
QM0 = deg2rad([30 60 -45]);
KP_EE = 2.0;  K_TH = 1.0;  LAM = 0.02;  T_MOVE = 8;
[~, ~, d0] = rns_controller(0, [0 0 0], QM0, LEN, MS, IN, m0, I0, b0, ...
    KP_EE, K_TH, LAM, 0, [0 0], [0 0], T_MOVE);
XS = d0(2:3).';
goals = struct('A', XS + [-0.8 -0.5], 'B', XS + [-0.6 0]);
fprintf('start end effector (%.4f, %.4f) m\n', XS);

% path, method, stop time (exact RNS on path B blows up near 6.4 s: stop before)
cases = {'A', 0, 10; 'A', 1, 10; 'A', 2, 10; 'B', 0, 10; 'B', 1, 6.3; 'B', 2, 10};
mname = {'minnorm', 'exact', 'priority'};

build_rns_sm('rns_sm');
R = struct();
for c = 1:size(cases, 1)
    XG = goals.(cases{c, 1});
    METHOD = cases{c, 2};
    T_END = cases{c, 3};
    tic;
    out = sim('rns_sm');
    t = out.t_sm(:);
    Y = squeeze(out.Y_sm);
    if size(Y, 1) ~= numel(t), Y = Y.'; end     % N x 15
    key = sprintf('%s_%s', cases{c, 1}, mname{METHOD + 1});
    R.(key) = struct('t', t, 'Y', Y);
    fprintf('path %s %-8s: max|th0| = %8.4f deg, th0(end) = %+8.4f deg, max|qd_m| = %.3f rad/s (%.1f s wall)\n', ...
        cases{c, 1}, mname{METHOD + 1}, rad2deg(max(abs(Y(:, 3)))), rad2deg(Y(end, 3)), ...
        max(abs(Y(:, 10:12)), [], 'all'), toc);
end
save('rns_simscape_results.mat', '-struct', 'R');

figdir = fullfile(here, 'figures');
if ~isfolder(figdir), mkdir(figdir); end
open_system('rns_sm');
print('-srns_sm', '-dpng', '-r150', fullfile(figdir, 'rns_simscape_model.png'));
