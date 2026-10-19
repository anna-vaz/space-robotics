%STEP2_PD_SLEW  Project 4, step 2: a 90-degree slew with a PD controller designed
%   for a RIGID spacecraft, run on the flexible one.
%
%   A. Rigid design: u = Kp (theta_r - theta) + Kd (theta_r_dot - theta_dot) + J theta_r_dd.
%      What does the collocated PD do to the panel mode? (closed-loop poles vs gain)
%   B. Three ways to command the same 90-degree slew (torque limit 1 N m):
%        1. naive: step command, PD only (saturates)
%        2. bang-bang reference + feedforward (time-optimal for a rigid body)
%        3. smooth quintic reference + feedforward, same peak torque
%      Rigid model says 2 and 3 track perfectly. The flexible model says otherwise.
%   C. Residual vibration vs slew time for both profiles, with the hand formula.

clear; clc; close all
here = fileparts(mfilename('fullpath'));
figdir = fullfile(here, 'figures');
p = flex_params();
m = flex_model(p);
J = p.J_total;  ndof = size(m.M,1);
hz = @(w) w/(2*pi);  arcsec = 180/pi*3600;

%% A. rigid PD design and its effect on the flexible mode
fn = 0.05;  zc = 0.7;                       % rigid closed-loop bandwidth [Hz], damping
wn = 2*pi*fn;
ctrl.Kp = J*wn^2;  ctrl.Kd = 2*zc*J*wn;
fprintf('A. Rigid PD design: f_n = %.2f Hz, zeta = %.1f  ->  Kp = %.2f N m/rad, Kd = %.2f N m s/rad\n', ...
        fn, zc, ctrl.Kp, ctrl.Kd);

clp = @(Kp, Kd) eig(m.sys.A - m.sys.B*[Kp, zeros(1,ndof-1), Kd, zeros(1,ndof-1)]);
lam = clp(ctrl.Kp, ctrl.Kd);
fprintf('   closed-loop poles (flexible model), sorted by frequency:\n');
[~, o] = sort(abs(lam));  lam = lam(o);  lam = lam(imag(lam) >= 0);
for k = 1:min(5, numel(lam))
    fprintf('     %8.4f Hz   damping %.4f\n', hz(abs(lam(k))), -real(lam(k))/abs(lam(k)));
end

% sweep the bandwidth: what happens to the coupled panel mode?
fsweep = logspace(-3, 0.5, 600);
zmode = zeros(size(fsweep));  fmode = zmode;  maxre = zmode;
% follow the coupled panel mode by continuation: start from its open-loop value
% and, at each new gain, take the closed-loop pole nearest the previous one
% (a fixed frequency window would grab the rigid-body pole when it passes 0.46 Hz)
[V0, D0] = eig(m.Kq, m.M);  w0 = sqrt(abs(diag(D0)));
lprev = 1i*w0(abs(w0 - 2*pi*0.4634) < 0.01);  lprev = lprev(1);
for k = 1:numel(fsweep)
    w = 2*pi*fsweep(k);
    l = clp(J*w^2, 2*zc*J*w);
    maxre(k) = max(real(l));
    l = l(imag(l) > 0);
    [~, j] = min(abs(l - lprev));  lprev = l(j);
    zmode(k) = -real(lprev)/abs(lprev);  fmode(k) = hz(abs(lprev));
end
[zbest, kb] = max(zmode);
fprintf('   bandwidth sweep 0.001-3 Hz: closed loop always stable? %s (max Re = %.1e)\n', ...
        string(all(maxre < 1e-9)), max(maxre));
fprintf('   best damping of the coupled panel mode: %.3f at f_n = %.3f Hz (structural alone: %.3f)\n', ...
        zbest, fsweep(kb), p.zeta);
fprintf('   at very high gain (hub pinned) it tends to the cantilever mode: %.4f Hz, damping %.4f\n', fmode(end), zmode(end));
fprintf('   at f_n = %.2f Hz the panel mode has damping %.4f\n', fn, interp1(fsweep, zmode, fn));

%% B. three ways to slew 90 degrees
theta_f = pi/2;  u_ff_peak = 0.8;           % keep 0.2 N m for feedback
T_bb = 2*sqrt(theta_f*J/u_ff_peak);          % peak acceleration 4 theta_f / T^2
T_q  = sqrt(5.7735*theta_f*J/u_ff_peak);     % quintic peak acceleration 5.7735 theta_f / T^2
t_end = 250;
fprintf('\nB. 90-degree slew, |u| <= %.1f N m, feedforward peak %.1f N m\n', p.u_max, u_ff_peak);
fprintf('   bang-bang duration %.2f s, quintic duration %.2f s\n', T_bb, T_q);

cases = { 'step command, PD only', @(t) [theta_f; 0; 0],                      [],                  NaN;
          'bang-bang + feedforward', @(t) refff('bangbang', t, theta_f, T_bb, J), [T_bb/2, T_bb], T_bb;
          'quintic + feedforward',   @(t) refff('quintic',  t, theta_f, T_q,  J), T_q,             T_q };
tol_th = 1e-3*pi/180;  tol_tip = 1e-3;       % "settled": 0.001 deg (3.6 arcsec) and 1 mm
res = struct();
fprintf('   %-26s  end of slew   residual tip   hub jitter   settled (0.001 deg & 1 mm)\n', '');
for c = 1:size(cases,1)
    o = flex_simulate(m, p, ctrl, cases{c,2}, t_end, cases{c,3});
    err = o.theta - theta_f;
    bad = abs(err) > tol_th | abs(o.tip) > tol_tip;
    ts  = o.t(find(bad, 1, 'last') + 1);  if isempty(ts), ts = 0; end
    Te  = cases{c,4};
    if isnan(Te), Te = o.t(find(abs(err) > 1*pi/180, 1, 'last') + 1); end   % "arrived" within 1 deg
    w = o.t >= Te & o.t <= Te + 20;
    fprintf('   %-26s  %8.1f s   %8.1f mm   %7.1f arcsec   %7.1f s\n', cases{c,1}, Te, ...
            1e3*max(abs(o.tip(w))), arcsec*max(abs(err(w))), ts);
    res(c).o = o;  res(c).name = cases{c,1};  res(c).Te = Te;  res(c).ts = ts;
end
fprintf('   peak overshoot of the step command: %.2f deg\n', max(res(1).o.theta - theta_f)*180/pi);
fprintf('   (rigid model: cases 2 and 3 would track exactly -> settled at %.1f and %.1f s)\n', T_bb, T_q);
tau = 1/(p.zeta*2*pi*0.4634);
fprintf('   free decay of the panel mode: e-folding time 1/(zeta omega) = %.0f s\n', tau);

%% C. residual vibration vs slew duration (linear, no saturation needed for T >= T_bb)
% With the hub tracking well, each panel mode i is driven by the reference
% acceleration: eta'' + Omega^2 eta = -delta theta_r''. For a bang-bang
% acceleration +a, -a lasting T/2 each, the residual amplitude is
%     |eta|  = (delta a / Omega^2) * 4 sin^2(Omega T / 4)
% -> it vanishes when T is a multiple of 2 * (vibration period).
Ts = linspace(T_bb, 120, 400);
A_bb = zeros(size(Ts));  A_q = A_bb;
dt = 0.01;
for k = 1:numel(Ts)
    for typ = ["bangbang", "quintic"]
        T = Ts(k);
        tt = (0:dt:T + 30)';
        r = refff(char(typ), tt', theta_f, T, J);
        o = lsim_cl(m, ctrl, r', tt, ndof);
        a = max(abs(o(tt >= T)));
        if typ == "bangbang", A_bb(k) = a; else, A_q(k) = a; end
    end
end
% Hand formula. The PD is soft (0.05 Hz), so during the slew the spacecraft is
% almost FREE and the feedforward torque u = J a drives its coupled mode
% (mass-normalised shape v):  q'' + 2 zeta w q' + w^2 q = (v' B) u.
% A force +F for T/2 then -F for T/2: each switch starts a decaying oscillation
% e^{lambda t}, lambda = -zeta w + i w_d. Adding the three switches (+F at 0,
% -2F at T/2, +F at T), the vibration left at the end of the slew is
%     |q| = F / w^2 * |1 - e^{lambda T/2}|^2   ( = F/w^2 * 4 sin^2(w T/4) undamped)
% i.e. (almost) ZERO whenever T is a whole number of TWO vibration periods.
% The damping matters: over a 40-100 s slew the vibration started at t = 0
% decays by e^{-zeta w T} ~ 0.5-0.2, so it no longer cancels exactly. We use the
% closed-loop pole lambda (it includes the small damping the PD adds).
[V, D] = eig(m.Kq, m.M);  wv = sqrt(abs(diag(D)));
[~, j] = min(abs(wv - 2*pi*0.4634));  v = V(:,j) / sqrt(V(:,j)'*m.M*V(:,j));  wc = wv(j);
tip_of_q = m.sys.C(3, 1:ndof) * v;            % tip deflection per unit modal amplitude
lcl = clp(ctrl.Kp, ctrl.Kd);  [~, j] = min(abs(lcl - 1i*wc));  lcl = lcl(j);
a_of_T = 4*theta_f ./ Ts.^2;
A_hand = abs(tip_of_q * (v'*m.B)) * J*a_of_T / abs(lcl)^2 .* abs(1 - exp(lcl*Ts/2)).^2;
fprintf('\nC. Residual tip amplitude vs slew time (bang-bang)\n');
fd = hz(imag(lcl));
fprintf('   hand formula: minima at T = k * 2 / f_d, f_d = %.4f Hz (closed-loop panel mode) -> every %.2f s: %s ...\n', ...
        fd, 2/fd, sprintf('%.1f ', 2/fd*(10:13)));
[~, iz] = findpeaks(-A_bb);
fprintf('   simulated minima at T = %s s\n', sprintf('%.1f ', Ts(iz)));
fprintf('   mean spacing of simulated minima %.2f s\n', mean(diff(Ts(iz))));
fprintf('   simulated vs hand formula: max |difference| = %.2f mm (peak residual %.1f mm)\n', ...
        1e3*max(abs(A_bb - A_hand)), 1e3*max(A_bb));
fprintf('   quintic residual is below %.2f mm for all T >= %.0f s\n', 1e3*max(A_q), Ts(1));

save(fullfile(here, 'results_step2.mat'), 'res', 'ctrl', 'theta_f', 'T_bb', 'T_q', 'u_ff_peak');

%% figures
f = figure('Position', [80 80 1400 800]); theme(f, 'light');
cols = lines(3);
subplot(2,3,1); hold on; grid on
for c = 1:3, plot(res(c).o.t, res(c).o.theta*180/pi, 'Color', cols(c,:), 'LineWidth', 1.5); end
xlabel('time [s]'); ylabel('hub angle [deg]'); title('Slew of 90^\circ'); xlim([0 150])
legend({res.name}, 'Location', 'southeast');
subplot(2,3,2); hold on; grid on
for c = 1:3, plot(res(c).o.t, res(c).o.u, 'Color', cols(c,:), 'LineWidth', 1.2); end
xlabel('time [s]'); ylabel('torque [N m]'); title('Actuator torque (limit \pm1 N m)'); xlim([0 150])
subplot(2,3,3); hold on; grid on
for c = 1:3, plot(res(c).o.t, 1e3*res(c).o.tip, 'Color', cols(c,:), 'LineWidth', 1.0); end
xlabel('time [s]'); ylabel('tip deflection [mm]'); title('Solar panel tip deflection'); xlim([0 150])
subplot(2,3,4); hold on; grid on
for c = 2:3
    o = res(c).o;  w = o.t >= res(c).Te;
    semilogy(o.t(w), arcsec*abs(o.theta(w) - theta_f), 'Color', cols(c,:), 'LineWidth', 1.0);
end
set(gca, 'YScale', 'log'); xlim([35 150]); ylim([1e-2 1e3]); yline(3.6, 'k--', '0.001^\circ', 'LabelHorizontalAlignment','left');
xlabel('time [s]'); ylabel('|pointing error| [arcsec]'); title('Pointing error after the slew');
legend({res(2:3).name}, 'Location', 'northeast');
subplot(2,3,5); hold on; grid on
semilogx(fsweep, zmode, 'LineWidth', 1.8); set(gca, 'XScale', 'log');
xline(fn, 'k--', 'design'); xline(hz(m.omega_c(1)), ':', 'panel 0.40 Hz');
xlabel('PD bandwidth f_n [Hz]'); ylabel('damping ratio of coupled panel mode');
title('Collocated PD: damping it adds to the panel mode');
subplot(2,3,6); hold on; grid on
plot(Ts, 1e3*A_bb, 'Color', cols(2,:), 'LineWidth', 1.8);
plot(Ts, 1e3*A_hand, 'k--', 'LineWidth', 1.0);
plot(Ts, 1e3*A_q, 'Color', cols(3,:), 'LineWidth', 1.8);
xlabel('slew duration T [s]'); ylabel('residual tip amplitude [mm]');
title('Residual vibration vs slew time');
legend('bang-bang (simulated)', 'bang-bang (hand formula)', 'quintic (simulated)', 'Location', 'northeast');
exportgraphics(f, fullfile(figdir, 'step2_pd_slew.png'), 'Resolution', 150);
fprintf('\nsaved %s\n', fullfile(figdir, 'step2_pd_slew.png'));

%% local functions
function r = refff(type, t, theta_f, T, J)
% reference [theta_r; theta_r_dot; u_ff] with rigid-body feedforward u_ff = J theta_r_dd
[th, thd, thdd] = slew_profile(type, t, theta_f, T);
r = [th; thd; J*thdd];
end

function tip = lsim_cl(m, ctrl, r, t, ndof)
% linear closed loop (no saturation), inputs [theta_r, theta_r_dot, u_ff], output tip
A = m.sys.A;  B = m.sys.B;
K = [ctrl.Kp, zeros(1,ndof-1), ctrl.Kd, zeros(1,ndof-1)];
Acl = A - B*K;
Bcl = B*[ctrl.Kp, ctrl.Kd, 1];
y = lsim(ss(Acl, Bcl, m.sys.C(3,:), 0), r, t, 'foh');
tip = y;
end
