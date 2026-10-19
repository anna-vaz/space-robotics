function fe = flex_fe_model(p, n_el)
%FLEX_FE_MODEL  Independent finite-element model of the same spacecraft.
%
%   Used only to CHECK flex_model.m. Different method, different coordinates:
%   each panel is cut into n_el Euler-Bernoulli beam elements (cubic Hermite,
%   consistent mass), and the unknowns are the INERTIAL transverse displacement
%   v and slope v' at every node (rigid rotation + bending together; no mode
%   shapes, no coupling coefficients). The panel root is welded to the hub:
%       v_root = r * theta,    v'_root = theta.
%   The hub adds only J_hub * theta_dd.
%
%   Returns mass/stiffness matrices for the free spacecraft (dofs: theta, then
%   the free nodes of both panels) and for one clamped panel (hub held fixed).

h  = p.L / n_el;
EI = p.EI;  rho = p.rho;
ke = EI/h^3 * [ 12   6*h   -12   6*h;
                6*h  4*h^2 -6*h  2*h^2;
               -12  -6*h    12  -6*h;
                6*h  2*h^2 -6*h  4*h^2];
me = rho*h/420 * [ 156    22*h    54    -13*h;
                   22*h   4*h^2   13*h  -3*h^2;
                   54     13*h    156   -22*h;
                  -13*h  -3*h^2  -22*h   4*h^2];

nd = 2*(n_el + 1);                    % dofs of one panel incl. the root node
Kp = zeros(nd);  Mp = zeros(nd);
for e = 1:n_el
    idx = 2*e-1 : 2*e+2;
    Kp(idx,idx) = Kp(idx,idx) + ke;
    Mp(idx,idx) = Mp(idx,idx) + me;
end

% clamped panel: remove root dofs (1,2)
fe.K_clamped = Kp(3:end,3:end);
fe.M_clamped = Mp(3:end,3:end);

% free spacecraft: panel dofs = T * [theta; free dofs]
nf = nd - 2;
T  = zeros(nd, 1 + nf);
T(1,1) = p.r;  T(2,1) = 1;            % root welded to the hub
T(3:end, 2:end) = eye(nf);
Kt = T'*Kp*T;  Mt = T'*Mp*T;          % one panel, in [theta; its free dofs]

% assemble hub + two identical panels: dofs [theta; panel 1 free; panel 2 free]
N = 1 + 2*nf;
K = zeros(N);  M = zeros(N);
M(1,1) = p.J_hub;
for k = 1:2
    g = [1, 1 + (k-1)*nf + (1:nf)];
    K(g,g) = K(g,g) + Kt;
    M(g,g) = M(g,g) + Mt;
end
fe.K = K;  fe.M = M;  fe.n_el = n_el;
end
