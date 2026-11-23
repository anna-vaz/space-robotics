%RUN_SIMSCAPE_CROSSCHECK  Build and run the Simscape Multibody model of project 1.
%
%   Same parameters, reference and PD gains as params.py / simulation.py.
%   Saves the time histories to simscape_results.mat for compare_simscape.py.
%
%   Run from this folder:
%     matlab -batch "run_simscape_crosscheck"

clear; clc;
here = fileparts(mfilename('fullpath'));
cd(here);

% ---- parameters (params.py) ------------------------------------------------------
m0 = 100;  a0 = 1;  I0 = m0*(a0^2 + a0^2)/12;   % base: square plate
b0 = [a0/2, 0];                                % shoulder in the base frame
l1 = 1; l2 = 1; m1 = 10; m2 = 10;              % links: uniform rods
I1 = m1*l1^2/12;  I2 = m2*l2^2/12;

% ---- reference and controller (simulation.py) -------------------------------------
Q_START = deg2rad([0 0]);  Q_GOAL = deg2rad([90 -90]);
T_MOVE = 10;  T_END = 15;
KP = [40 20];  KD = [30 10];

% ---- build + simulate ---------------------------------------------------------------
build_free_floating_sm('free_floating_sm');
tic;
out = sim('free_floating_sm');
fprintf('simulated %.0f s in %.1f s wall time\n', T_END, toc);

t = out.t_sm(:);
Y = squeeze(out.Y_sm);
if size(Y, 1) ~= numel(t), Y = Y.'; end           % -> N x 12
q = Y(:, 1:5);  qd = Y(:, 6:10);  tau = Y(:, 11:12);

% ---- sanity check of the sensor-port order against Simscape's own log --------------
simlog = out.simlog;
bj = simlog.child(simlog.childIds{contains(simlog.childIds, 'Base_joint')});
fprintf('planar joint log nodes: %s\n', strjoin(bj.childIds, ' '));
chk = {'Px', 'p', 1, 'm'; 'Py', 'p', 2, 'm'; 'Rz', 'q', 3, 'rad'};
for k = 1:3
    s = bj.child(chk{k,1}).child(chk{k,2}).series;
    v = interp1(s.time, s.values(chk{k,4}), t, 'pchip');
    fprintf('log %s.%s vs sensor column %d: max diff %.1e\n', chk{k,1}, chk{k,2}, chk{k,3}, max(abs(v - q(:, chk{k,3}))));
end

fprintf('final base attitude th0 = %.4f deg, base position (%.4f, %.4f) m\n', ...
    rad2deg(q(end,3)), q(end,1), q(end,2));
fprintf('final joints q1 = %.3f deg, q2 = %.3f deg\n', rad2deg(q(end,4)), rad2deg(q(end,5)));

save('simscape_results.mat', 't', 'q', 'qd', 'tau');

% ---- diagram picture for the README ----------------------------------------------
figdir = fullfile(here, 'figures');
if ~isfolder(figdir), mkdir(figdir); end
open_system('free_floating_sm');
print('-sfree_floating_sm', '-dpng', '-r150', fullfile(figdir, 'simscape_model.png'));
