function p = flex_params()
%FLEX_PARAMS  Spacecraft with two flexible solar panels (single-axis slew).
%
%   Rigid hub (the satellite bus) of half-width r, with two identical solar
%   panels on opposite sides. Each panel is a uniform Euler-Bernoulli beam
%   clamped to the hub at its root. Only rotation about one axis (the slew
%   axis, normal to the panels' bending plane) is modelled.
%
%   Numbers are of the order of a small Earth-observation satellite with
%   deployed arrays; the panel stiffness is chosen so the first cantilever
%   frequency is 0.4 Hz, low enough that a fast slew excites it.

p.J_hub = 150;        % hub (bus) moment of inertia about the slew axis [kg m^2]
p.r     = 0.5;        % hub half-width = distance from axis to panel root [m]
p.L     = 3.0;        % panel length [m]
p.rho   = 2.0;        % panel mass per unit length [kg/m]   (6 kg per panel)
p.f1    = 0.4;        % target first cantilever frequency [Hz]
p.zeta  = 0.005;      % modal damping ratio (typical for deployed arrays)
p.n_modes = 3;        % cantilever modes kept per panel

% Choose EI so that the first cantilever frequency is f1:
%   omega_1 = (beta_1 L)^2 sqrt(EI / (rho L^4)),  beta_1 L = 1.8751
bL1  = 1.875104068711961;
p.EI = p.rho * p.L^4 * (2*pi*p.f1 / bL1^2)^2;     % bending stiffness [N m^2]

p.u_max = 1.0;        % actuator torque limit [N m]

% rigid-body inertia of one panel about the slew axis: rho * int_r^{r+L} s^2 ds
p.J_panel = p.rho * ((p.r + p.L)^3 - p.r^3) / 3;
p.J_total = p.J_hub + 2*p.J_panel;                 % whole spacecraft, rigid
end
