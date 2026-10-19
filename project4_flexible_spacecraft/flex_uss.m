function [G, info] = flex_uss(p, n_modes, kEI_range, gC_range)
%FLEX_USS  Uncertain model of the spacecraft for robust control design.
%
%   Only the panel motion that the torque can excite is kept: both panels bending
%   in the same rotational sense. With eta_s = (eta_1 + eta_2)/sqrt(2):
%       [J        sqrt2 d'] [theta_dd]   [0              ]   [u]
%       [sqrt2 d  I       ] [eta_s_dd] + [C eta_s_d + K eta_s] = [0]
%   (the opposite-sense combination is uncontrollable and unobservable; step 1).
%
%   Uncertainty (Robust Control Toolbox ureal):
%     kEI : panel bending stiffness / nominal. Mode shapes do not depend on EI,
%           only the frequencies do, so K = kEI * K0 exactly (Omega ~ sqrt(EI)).
%     gC  : panel damping / nominal (structural damping is poorly known).
%
%   G: uss, input u [N m], outputs [theta; theta_dot; tip of panel 1].
if nargin < 3, kEI_range = [0.5 1.5]; end
if nargin < 4, gC_range = [0.4 4]; end
m  = flex_model(p, n_modes);
n  = m.n;
kEI = ureal('kEI', 1, 'Range', kEI_range);
gC  = ureal('gC',  1, 'Range', gC_range);
Ms = [p.J_total, sqrt(2)*m.d';  sqrt(2)*m.d, eye(n)];
K0 = diag(m.omega_c.^2);  C0 = diag(2*p.zeta*m.omega_c);
Mi = inv(Ms);
Kq = blkdiag(0, kEI*K0);
Cq = blkdiag(0, gC*C0);
nd = 1 + n;
A = [zeros(nd), eye(nd);  -Mi*Kq, -Mi*Cq];
B = [zeros(nd,1); Mi*[1; zeros(n,1)]];
C = [1, zeros(1, 2*nd-1);
     zeros(1, nd), 1, zeros(1, nd-1);
     0, m.tip'/sqrt(2), zeros(1, nd)];
G = ss(A, B, C, 0);
G.InputName = 'u';  G.OutputName = {'theta', 'theta_dot', 'tip'};
info = struct('Ms', Ms, 'K0', K0, 'C0', C0, 'm', m);
end
