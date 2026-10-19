function out = flex_simulate(m, p, ctrl, ref, t_end, breaks)
%FLEX_SIMULATE  Closed-loop slew of the flexible spacecraft (with torque limit).
%
%   ctrl.Kp, ctrl.Kd  : PD gains acting on the hub angle and rate (collocated)
%   ctrl.K2 (optional): extra LTI feedback (ss) driven by the same errors
%                       y = [theta_r - theta; theta_r_dot - theta_dot]
%                       (the H-infinity / mu controllers of step 4 add this to the PD)
%   ref(t)            : returns [theta_r, theta_r_dot, u_ff] (reference + feedforward)
%   breaks            : times where ref is not smooth (integration restarts there,
%                       so ode45 never steps across a jump in the acceleration)
%
%   u = sat( Kp y1 + Kd y2 + K2(y) + u_ff ),  |u| <= u_max
%
%   Returns t, theta, theta_dot, tip deflection (panel 1), torque u, plant state x.
A = m.sys.A;  B = m.sys.B;  C = m.sys.C;
ndof = size(m.M, 1);
nx = size(A, 1);
if isfield(ctrl, 'K2') && ~isempty(ctrl.K2)
    [Ak, Bk, Ck, Dk] = ssdata(ctrl.K2);
else
    Ak = zeros(0);  Bk = zeros(0, 2);  Ck = zeros(1, 0);  Dk = zeros(1, 2);
end
nk = size(Ak, 1);
umax = p.u_max;
law = @(t, x) torque(t, x, ctrl, ref, ndof, nx, Ck, Dk, umax);
rhs = @(t, x) [A*x(1:nx) + B*law(t, x);  Ak*x(nx+1:end) + Bk*errs(t, x, ref, ndof)];

opts = odeset('RelTol', 1e-9, 'AbsTol', 1e-12);
edges = unique([0, breaks(breaks > 0 & breaks < t_end), t_end]);
x0 = zeros(nx + nk, 1);
T = [];  X = [];
for k = 1:numel(edges) - 1
    tk = edges(k):0.02:edges(k+1);
    if tk(end) < edges(k+1), tk = [tk, edges(k+1)]; end
    [tt, xx] = ode45(rhs, tk, x0, opts);
    if numel(tk) == 2, xx = xx([1 end], :); tt = tt([1 end]); end
    T = [T; tt(1:end-1)];  X = [X; xx(1:end-1, :)];  %#ok<AGROW>
    x0 = xx(end, :)';
end
T = [T; edges(end)];  X = [X; x0'];
y = X(:, 1:nx) * C';
u = zeros(size(T));
for k = 1:numel(T), u(k) = law(T(k), X(k,:)'); end
out = struct('t', T, 'x', X(:, 1:nx), 'theta', y(:,1), 'theta_dot', y(:,2), 'tip', y(:,3), 'u', u);
end

function y = errs(t, x, ref, ndof)
r = ref(t);
y = [r(1) - x(1); r(2) - x(ndof+1)];
end

function u = torque(t, x, ctrl, ref, ndof, nx, Ck, Dk, umax)
r = ref(t);
y = [r(1) - x(1); r(2) - x(ndof+1)];
u = ctrl.Kp*y(1) + ctrl.Kd*y(2) + r(3) + Ck*x(nx+1:end) + Dk*y;
u = min(max(u, -umax), umax);
end
