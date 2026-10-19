function out = flex_simulate(m, p, ctrl, ref, t_end, breaks)
%FLEX_SIMULATE  Closed-loop slew of the flexible spacecraft (with torque limit).
%
%   ctrl.Kp, ctrl.Kd : PD gains acting on the hub angle and rate (collocated)
%   ref(t)           : returns [theta_r, theta_r_dot, u_ff] (reference + feedforward)
%   breaks           : times where ref is not smooth (integration restarts there,
%                      so ode45 never steps across a jump in the acceleration)
%
%   u = sat( Kp (theta_r - theta) + Kd (theta_r_dot - theta_dot) + u_ff ),  |u| <= u_max
%
%   Returns t, theta, theta_dot, tip deflection (panel 1), torque u, full state x.
A = m.sys.A;  B = m.sys.B;  C = m.sys.C;
ndof = size(m.M, 1);
umax = p.u_max;
law = @(t, x) torque(t, x, ctrl, ref, ndof, umax);
rhs = @(t, x) A*x + B*law(t, x);

opts = odeset('RelTol', 1e-9, 'AbsTol', 1e-12);
edges = unique([0, breaks(breaks > 0 & breaks < t_end), t_end]);
x0 = zeros(size(A,1), 1);
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
y = X * C';
u = zeros(size(T));
for k = 1:numel(T), u(k) = law(T(k), X(k,:)'); end
out = struct('t', T, 'x', X, 'theta', y(:,1), 'theta_dot', y(:,2), 'tip', y(:,3), 'u', u);
end

function u = torque(t, x, ctrl, ref, ndof, umax)
r = ref(t);
u = ctrl.Kp*(r(1) - x(1)) + ctrl.Kd*(r(2) - x(ndof+1)) + r(3);
u = min(max(u, -umax), umax);
end
