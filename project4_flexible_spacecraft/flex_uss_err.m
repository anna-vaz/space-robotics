function [G, info] = flex_uss_err(p, n_modes, kEI_range, gC_range)
%FLEX_USS_ERR  Uncertain model written in TRACKING-ERROR coordinates.
%
%   During a slew the reference obeys theta_r_dd = a_r and the feedforward torque
%   is J a_r (rigid design). Write theta = theta_r + e and u = J a_r + u_fb + d.
%   Substituting into the (symmetric-mode) equations of motion:
%       J e_dd + sqrt2 d' eta_dd                 = u_fb + d
%       eta_dd + C eta_d + K eta + sqrt2 d e_dd  = -sqrt2 d a_r
%   The rigid part of the feedforward cancels exactly; what is left is the
%   panels being shaken by the commanded acceleration a_r. This is the input
%   a robust controller has to reject.
%
%   G: uss with inputs [u_fb; d; a_r], outputs [e; e_dot; tip of panel 1].
if nargin < 3, kEI_range = [0.5 1.5]; end
if nargin < 4, gC_range = [0.4 4]; end
m  = flex_model(p, n_modes);
n  = m.n;
kEI = ureal('kEI', 1, 'Range', kEI_range);
gC  = ureal('gC',  1, 'Range', gC_range);
Ms = [p.J_total, sqrt(2)*m.d';  sqrt(2)*m.d, eye(n)];
K0 = diag(m.omega_c.^2);  C0 = diag(2*p.zeta*m.omega_c);
Mi = inv(Ms);
nd = 1 + n;
A = [zeros(nd), eye(nd);  -Mi*blkdiag(0, kEI*K0), -Mi*blkdiag(0, gC*C0)];
Bu = Mi*[1; zeros(n,1)];
Ba = Mi*[0; -sqrt(2)*m.d];
B = [zeros(nd,3); Bu, Bu, Ba];
C = [1, zeros(1, 2*nd-1);
     zeros(1, nd), 1, zeros(1, nd-1);
     0, m.tip'/sqrt(2), zeros(1, nd)];
G = ss(A, B, C, 0);
G.InputName = {'ufb', 'd', 'ar'};  G.OutputName = {'e', 'e_dot', 'tip'};
info = struct('Ms', Ms, 'm', m);
end
