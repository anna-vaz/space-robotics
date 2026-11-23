%RUN_FIXED_BASE  The same 2-link arm on a FIXED base (bolted to the ground), in Simscape.
%
%   Same parameters, reference and PD gains as the free-floating run, so the two can be
%   compared directly (compare_fixed_vs_free.py). Saves fixed_base_results.mat.
%
%   Run from this folder:   matlab -batch "run_fixed_base"

clear; clc;
here = fileparts(mfilename('fullpath'));
cd(here);

m0 = 100;  a0 = 1;  I0 = m0*(a0^2 + a0^2)/12;  b0 = [a0/2, 0];
l1 = 1; l2 = 1; m1 = 10; m2 = 10;  I1 = m1*l1^2/12;  I2 = m2*l2^2/12;
Q_START = deg2rad([0 0]);  Q_GOAL = deg2rad([90 -90]);
T_MOVE = 10;  T_END = 15;
KP = [40 20];  KD = [30 10];

build_free_floating_sm('fixed_base_sm', true);
out = sim('fixed_base_sm');
t = out.t_sm(:);
Y = squeeze(out.Y_sm);
if size(Y, 1) ~= numel(t), Y = Y.'; end
q = Y(:, 1:5);  qd = Y(:, 6:10);  tau = Y(:, 11:12);
fprintf('fixed base: final joints q1 = %.3f deg, q2 = %.3f deg; max |tau| = %.3f / %.3f N m\n', ...
    rad2deg(q(end,4)), rad2deg(q(end,5)), max(abs(tau)));
save('fixed_base_results.mat', 't', 'q', 'qd', 'tau');

figdir = fullfile(here, 'figures');
if ~isfolder(figdir), mkdir(figdir); end
open_system('fixed_base_sm');
print('-sfixed_base_sm', '-dpng', '-r150', fullfile(figdir, 'fixed_base_model.png'));
