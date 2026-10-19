%STEP1_MODEL  Project 4, step 1: model of a spacecraft with flexible solar panels.
%
%   A. Parameters and hand checks (inertia, stiffness).
%   B. Cantilever frequencies: analytical vs finite elements.
%   C. Free spacecraft: hybrid model (rigid hub + n cantilever modes) vs an
%      independent finite-element model, as the number of modes grows.
%   D. How much of each panel's inertia each mode carries (completeness check).
%   E. Frequency response torque -> attitude: flexible vs rigid; figures.

clear; clc; close all
here = fileparts(mfilename('fullpath'));
figdir = fullfile(here, 'figures');
if ~exist(figdir, 'dir'), mkdir(figdir); end

p = flex_params();
m = flex_model(p);
hz = @(w) w/(2*pi);

%% A. parameters
fprintf('A. Parameters\n');
fprintf('   hub inertia %.0f kg m^2, two panels %.1f m x %.1f kg/m (%.0f kg each), EI = %.2f N m^2\n', ...
        p.J_hub, p.L, p.rho, p.rho*p.L, p.EI);
fprintf('   one panel about the axis: rho*((r+L)^3 - r^3)/3 = %.2f kg m^2\n', p.J_panel);
fprintf('   whole spacecraft (rigid): %.2f kg m^2  -> panels are %.1f %% of it\n', ...
        p.J_total, 100*2*p.J_panel/p.J_total);

%% B. cantilever frequencies
fe = flex_fe_model(p, 40);
w_fe_c = sort(sqrt(eig(fe.K_clamped, fe.M_clamped)));
fprintf('\nB. Cantilever (hub held fixed) frequencies [Hz]\n');
fprintf('   mode   beta_i L    analytical   FE (40 elements)   rel. diff\n');
for i = 1:m.n
    fprintf('    %d    %8.5f   %10.6f   %14.6f     %8.1e\n', i, m.bL(i), hz(m.omega_c(i)), ...
            hz(w_fe_c(i)), w_fe_c(i)/m.omega_c(i) - 1);
end

%% C. free spacecraft: hybrid model vs finite elements
fe = flex_fe_model(p, 60);
w_fe = sort(sqrt(abs(eig(fe.K, fe.M))));
fprintf('\nC. Free spacecraft (hub free to rotate): finite elements, 60 elements per panel\n');
% a rigid rotation (theta = 1, every node at v = s, v' = 1) must bend nothing
% and have the inertia of the whole spacecraft
nodes = [p.r + (p.L/60)*(1:60); ones(1,60)];
v_rigid = [1; nodes(:); nodes(:)];
fprintf('   rigid rotation: |K v| / |K| = %.1e  (no elastic force)\n', norm(fe.K*v_rigid)/norm(fe.K));
fprintf('   rigid rotation: inertia v''M v = %.6f kg m^2 (formula %.6f)\n', ...
        v_rigid'*fe.M*v_rigid, p.J_total);
fprintf('   elastic frequencies [Hz], and relative error of the hybrid model with n modes per panel:\n');
fprintf('   FE [Hz]     n=1        n=2        n=3        n=5      shape\n');
nlist = [1 2 3 5];
W = nan(5, numel(nlist));
for k = 1:numel(nlist)
    mk = flex_model(p, nlist(k));
    w = sort(sqrt(abs(eig(mk.Kq, mk.M))));
    w = w(2:end);
    W(1:min(5,numel(w)), k) = w(1:min(5,numel(w)));
end
for i = 1:5
    fprintf('   %8.5f', hz(w_fe(i+1)));
    fprintf('  %+9.1e', W(i,:)/w_fe(i+1) - 1);
    if any(abs(w_fe(i+1) - m.omega_c) < 1e-3*w_fe(i+1)), fprintf('   uncoupled (hub still)\n');
    else, fprintf('   coupled (hub rotates)\n'); end
end
fprintf('   (NaN = mode not in a model that small. Uncoupled modes: the panels swing in\n');
fprintf('    opposite senses and the hub stays still, so they sit exactly at the cantilever values.)\n');

% single-mode hand check: coupled frequency^2 = omega_c^2 / (1 - 2 d1^2 / J)
w1_hand = m.omega_c(1) / sqrt(1 - 2*m.d(1)^2/p.J_total);
fprintf('   hand formula, 1 mode: omega_c1/sqrt(1 - 2 d1^2/J) = %.5f Hz\n', hz(w1_hand));

%% D. completeness: sum_i d_i^2 -> rho * int (r+x)^2 dx = J_panel
m10 = flex_model(p, 10);
fprintf('\nD. Rotational coupling d_i = rho*int (r+x) phi_i dx  [kg^0.5 m]\n');
fprintf('   mode   d_i       d_i^2 / J_panel   cumulative\n');
cum = cumsum(m10.d.^2)/p.J_panel;
for i = 1:5
    fprintf('    %d   %8.4f    %8.4f %%       %8.4f %%\n', i, m10.d(i), 100*m10.d(i)^2/p.J_panel, 100*cum(i));
end
fprintf('   10 modes: %.4f %% (must tend to 100 %%)\n', 100*cum(end));

%% E. frequency response  u -> theta
s_rig = tf(1, [p.J_total 0 0]);
G = m.sys;
fprintf('\nE. Transfer function torque -> hub angle\n');
Gm = minreal(G(1,1), 1e-9);
fprintf('   minimal realisation: %d of %d states left -> the %d uncoupled-mode states can be\n', ...
        order(Gm), order(G), order(G) - order(Gm));
fprintf('   neither moved by the torque nor seen in theta (they cancel exactly).\n');
zw = sort(abs(tzero(Gm)));  pw = sort(abs(pole(Gm)));
zw = unique(round(zw, 8));  pw = unique(round(pw(pw > 1e-6), 8));
fprintf('   zeros [Hz]:              %s\n', sprintf('%.4f   ', hz(zw)));
fprintf('   poles [Hz]: 0 (double)   %s\n', sprintf('%.4f   ', hz(pw)));
fprintf('   -> each zero (a cantilever frequency) sits just below a pole (a coupled frequency):\n');
fprintf('      pole/zero alternation, the signature of a collocated actuator + sensor.\n');

% ---- figures -----------------------------------------------------------------
x = linspace(0, p.L, 300);
f = figure('Position',[100 100 1300 420]); theme(f,'light');
subplot(1,3,1); hold on; grid on
for i = 1:m.n
    plot(x, m.phi{i}(x), 'LineWidth', 1.8, 'DisplayName', sprintf('mode %d: %.3f Hz', i, hz(m.omega_c(i))));
end
xlabel('distance from panel root x [m]'); ylabel('\phi_i(x)  [kg^{-1/2}]');
title('Cantilever mode shapes of one panel'); legend('Location','northwest');

% sketch of the first FREE mode of the whole spacecraft (FE eigenvector)
subplot(1,3,2); hold on; grid on
[V, D] = eig(fe.K, fe.M);  [~, ord] = sort(diag(D));  V = V(:, ord);
ns = numel(nodes)/2;  s = nodes(1,:);
lbl = {'uncoupled (hub still)', 'coupled (hub rotates)'};
for k = [2 3]
    v = V(:,k);  th = v(1);
    v1 = v(2:2:1+2*ns);  v2 = v(2+2*ns:2:end);
    sc = 0.6 / max(abs([v1; v2]));
    th = th*sc;  v1 = v1*sc;  v2 = v2*sc;
    c = [0 0 1; 0.85 0.33 0.1]; c = c(k-1,:);
    % hub square rotated by th, panel 1 to the right (+x), panel 2 to the left
    R = [cos(th) -sin(th); sin(th) cos(th)];
    sq = R*[p.r -p.r -p.r p.r p.r; p.r p.r -p.r -p.r p.r];
    plot(sq(1,:), sq(2,:), 'Color', c, 'LineWidth', 1.5, 'HandleVisibility','off');
    plot([p.r s], [p.r*th; v1]', 'Color', c, 'LineWidth', 2, ...
         'DisplayName', sprintf('%s: %.3f Hz', lbl{k-1}, hz(sqrt(D(ord(k),ord(k))))));
    plot(-[p.r s], -[p.r*th; v2]', 'Color', c, 'LineWidth', 2, 'HandleVisibility','off');
end
plot([-p.r-p.L, p.r+p.L], [0 0], 'k:', 'HandleVisibility','off');
xlabel('x [m]'); ylabel('y [m]  (deflection exaggerated)');
title('First two free modes (finite elements)'); legend('Location','south');
text(0, 0.85, 'hub', 'HorizontalAlignment','center');
xlim([-3.7 3.7]); ylim([-1.7 1.2])

subplot(1,3,3);
w = logspace(-1.5, 1.6, 40000)*2*pi;
magF = squeeze(bode(Gm, w));  magR = squeeze(bode(s_rig, w));
loglog(w/(2*pi), magF, 'LineWidth', 1.8); hold on; grid on
loglog(w/(2*pi), magR, '--k', 'LineWidth', 1.2);
xlabel('frequency [Hz]'); ylabel('|\theta / u|  [rad / N m]');
title('Torque \rightarrow hub angle'); legend('flexible', 'rigid  1/(J s^2)', 'Location','southwest');
exportgraphics(f, fullfile(figdir, 'step1_model.png'), 'Resolution', 150);
fprintf('\nsaved %s\n', fullfile(figdir, 'step1_model.png'));
