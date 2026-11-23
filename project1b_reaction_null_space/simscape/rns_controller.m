function [qd_m, qdd_m, info] = rns_controller(t, q_b, q_m, LEN, MS, IN, m0, I0, b0, ...
    KP_EE, K_TH, LAM, METHOD, XS, XG, T_MOVE)
%RNS_CONTROLLER  Joint-rate command of project 1b (same law as path_tracking.py).
%
%   Inputs: time, MEASURED base pose q_b = [x0 y0 th0] (from Simscape), joint angles q_m. Outputs: joint rates qd_m, their time derivative qdd_m (needed to
%   drive the Simscape joints by motion) and info = [det A; end-effector x; y].
%
%   METHOD: 0 = min-norm (base ignored), 1 = exact RNS, 2 = task priority + damped RNS.
%   The inertia matrix is computed numerically from the body Jacobians here; it is the
%   controller's MODEL. The plant (how the base really reacts) is Simscape's.

q = [q_b(:); q_m(:)];
[qd_m, detA, xe, B] = law(q, t, LEN, MS, IN, m0, I0, b0, KP_EE, K_TH, LAM, METHOD, XS, XG, T_MOVE);
% time derivative of the law along the motion (central difference). The base rates here
% are the controller's own zero-momentum prediction B*qd_m, so the feedforward does not
% depend on the plant's instantaneous velocity response.
qd = [B*qd_m; qd_m];
h = 1e-5;
qp = law(q + h*qd, t + h, LEN, MS, IN, m0, I0, b0, KP_EE, K_TH, LAM, METHOD, XS, XG, T_MOVE);
qn = law(q - h*qd, t - h, LEN, MS, IN, m0, I0, b0, KP_EE, K_TH, LAM, METHOD, XS, XG, T_MOVE);
qdd_m = (qp - qn) / (2*h);
info = [detA; xe];
end

function [qd_m, detA, xe, B] = law(q, t, LEN, MS, IN, m0, I0, b0, KP_EE, K_TH, LAM, METHOD, XS, XG, T_MOVE)
[M, xe, Je] = kin(q, LEN, MS, IN, m0, I0, b0);
B = -M(1:3, 1:3) \ M(1:3, 4:6);            % base rates per unit joint rate (zero momentum)
b = B(3, :);                               % reaction row: w0 = b qd_m
Js = Je(:, 4:6) + Je(:, 1:3) * B;          % generalized Jacobian J*
% straight-line quintic reference
x = min(max(t / T_MOVE, 0), 1);
s = 10*x^3 - 15*x^4 + 6*x^5;
ds = 0;
if t >= 0 && t < T_MOVE, ds = (30*x^2 - 60*x^3 + 30*x^4) / T_MOVE; end
xd = XS(:) + s*(XG(:) - XS(:));
vd = ds*(XG(:) - XS(:));
v = vd + KP_EE*(xd - xe);
n = cross(Js(1, :).', Js(2, :).');          % null space of J*
detA = b * n;
qd0 = pinv(Js) * v;
w_c = -K_TH * q(3);
if METHOD == 0
    sn = 0;
elseif METHOD == 1
    sn = (w_c - b*qd0) / detA;
else
    sn = detA * (w_c - b*qd0) / (detA^2 + LAM^2);
end
qd_m = qd0 + sn*n;
end

function [M, xe, Je] = kin(q, LEN, MS, IN, m0, I0, b0)
% Mass matrix from the body Jacobians, M = sum(m Jv'Jv + I Jw'Jw), and the
% end-effector position + Jacobian. q = [x0 y0 th0 q1 q2 q3].
th = q(3);
phi = th + cumsum(q(4:6));
R = [cos(th) -sin(th); sin(th) cos(th)];
j1 = q(1:2) + R*b0(:);
dj1 = R*[-b0(2); b0(1)];                   % d(shoulder)/d th0
Jv0 = [eye(2), zeros(2, 4)];
Jw0 = [0 0 1 0 0 0];
M = m0*(Jv0.'*Jv0) + I0*(Jw0.'*Jw0);
xe = zeros(2, 1);
Je = zeros(2, 6);
for i = 1:4                                % i = 1..3 link CoMs, i = 4 end effector
    Jv = [eye(2), dj1, zeros(2, 3)];
    p = j1;
    last = min(i, 3);
    for k = 1:last
        lever = LEN(k);
        if i <= 3 && k == i, lever = LEN(k)/2; end
        nk = [-sin(phi(k)); cos(phi(k))];
        p = p + lever*[cos(phi(k)); sin(phi(k))];
        Jv(:, 3) = Jv(:, 3) + lever*nk;
        Jv(:, 4:3+k) = Jv(:, 4:3+k) + lever*nk*ones(1, k);
    end
    if i <= 3
        Jw = [0 0 1, ones(1, i), zeros(1, 3-i)];
        M = M + MS(i)*(Jv.'*Jv) + IN(i)*(Jw.'*Jw);
    else
        xe = p;
        Je = Jv;
    end
end
end
