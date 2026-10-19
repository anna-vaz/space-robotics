function m = flex_model(p, n_modes)
%FLEX_MODEL  Hybrid (rigid hub + cantilever modes) model of the spacecraft.
%
%   Panel k deflects as  w_k(x,t) = sum_i phi_i(x) eta_ki(t),   0 <= x <= L,
%   where phi_i are the clamped-free (cantilever) beam modes, normalised so
%   that  rho * int phi_i^2 dx = 1.  A panel element at distance r + x from the
%   axis moves with transverse velocity (r + x) theta_dot + w_dot. Lagrange's
%   equations then give, with q = [theta; eta_1 (panel 1); eta_2 (panel 2)],
%
%       [ J      d'  d' ] [theta_dd]   [0          ]   [u]
%       [ d      I   0  ] [eta1_dd ] + [C eta1_d + K eta1] = [0]
%       [ d      0   I  ] [eta2_dd ]   [C eta2_d + K eta2]   [0]
%
%   J  = J_total (whole spacecraft treated as rigid),
%   d_i = rho * int (r + x) phi_i(x) dx   (rotational coupling of mode i),
%   K  = diag(omega_i^2),  C = diag(2 zeta omega_i).
%
%   Returns struct m with fields M, Kq, Cq, B (force input), the cantilever
%   frequencies, coupling coefficients, mode-shape handles, and a state-space
%   model sys with x = [q; q_dot], input u (torque), outputs
%   [theta; theta_dot; tip deflection of panel 1].

if nargin < 2, n_modes = p.n_modes; end
n = n_modes;

% ---- cantilever eigenvalues: roots of 1 + cos(bL) cosh(bL) = 0 -------------
bL = zeros(n,1);
for i = 1:n
    guess = (2*i - 1)*pi/2;                       % asymptotic value
    bL(i) = fzero(@(z) 1 + cos(z).*cosh(z), guess + [-0.6 0.6]);
end
beta  = bL / p.L;
omega = beta.^2 * sqrt(p.EI / p.rho);             % cantilever frequencies [rad/s]

% ---- mode shapes, normalised to rho * int phi^2 = 1 -------------------------
phi  = cell(n,1);
d    = zeros(n,1);
tip  = zeros(n,1);
for i = 1:n
    b = beta(i);  Z = bL(i);
    % textbook form: cosh(bx) - cos(bx) - s (sinh(bx) - sin(bx)),
    %                s = (cosh Z + cos Z) / (sinh Z + sin Z).
    % cosh - s sinh cancels catastrophically for higher modes (cosh(30) ~ 5e12),
    % so rewrite it with exponentials that never overflow:
    %   cosh(bx) - s sinh(bx) = (1-s)/2 e^{bx} + (1+s)/2 e^{-bx},
    %   1 - s = 2 e^{-Z} (sin Z - cos Z - e^{-Z}) / den,
    %   den   = 1 - e^{-2Z} + 2 e^{-Z} sin Z.
    den = 1 - exp(-2*Z) + 2*exp(-Z)*sin(Z);
    oms = 2*exp(-Z)*(sin(Z) - cos(Z) - exp(-Z)) / den;        % 1 - s
    s   = 1 - oms;
    raw = @(x) (sin(Z) - cos(Z) - exp(-Z))/den * exp(b*x - Z) ...
             + (1 + s)/2 * exp(-b*x) - cos(b*x) + s*sin(b*x);
    N = sqrt(p.rho * integral(@(x) raw(x).^2, 0, p.L, 'AbsTol',1e-13,'RelTol',1e-12));
    phi{i} = @(x) raw(x) / N;
    d(i)   = p.rho * integral(@(x) (p.r + x).*phi{i}(x), 0, p.L, 'AbsTol',1e-13,'RelTol',1e-12);
    tip(i) = phi{i}(p.L);
end

% ---- matrices ---------------------------------------------------------------
I  = eye(n);  Z = zeros(n);
M  = [p.J_total, d', d';  d, I, Z;  d, Z, I];
Kn = diag(omega.^2);
Cn = diag(2*p.zeta*omega);
Kq = blkdiag(0, Kn, Kn);
Cq = blkdiag(0, Cn, Cn);
B  = [1; zeros(2*n,1)];

ndof = 1 + 2*n;
A  = [zeros(ndof), eye(ndof);  -M\Kq, -M\Cq];
Bs = [zeros(ndof,1); M\B];
Cs = [1, zeros(1, 2*ndof-1);                                    % theta
      zeros(1,ndof), 1, zeros(1,ndof-1);                        % theta_dot
      0, tip', zeros(1,n), zeros(1,ndof)];                      % tip deflection, panel 1
sys = ss(A, Bs, Cs, 0, 'StateName', [{'theta'}, compose('eta1_%d',1:n), compose('eta2_%d',1:n), ...
         {'theta_dot'}, compose('eta1_%d_dot',1:n), compose('eta2_%d_dot',1:n)], ...
         'InputName','u', 'OutputName',{'theta','theta_dot','tip1'});

m = struct('M',M,'Kq',Kq,'Cq',Cq,'B',B,'omega_c',omega,'bL',bL,'d',d, ...
           'tip',tip,'phi',{phi},'n',n,'sys',sys);
end
