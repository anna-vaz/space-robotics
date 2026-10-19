function [th, thd, thdd] = slew_profile(type, t, theta_f, T)
%SLEW_PROFILE  Rest-to-rest reference for a slew of theta_f [rad] in T [s].
%   type = 'bangbang' : constant +a for T/2, then -a (time-optimal for a rigid body)
%          'quintic'  : 10 s^3 - 15 s^4 + 6 s^5 (zero velocity AND acceleration at ends)
%   Returns angle, rate and acceleration at times t (any shape). After T it holds theta_f.
s  = min(max(t / T, 0), 1);
in = (t >= 0) & (t <= T);
switch type
    case 'bangbang'
        a = 4*theta_f / T^2;                      % peak acceleration
        first = s <= 0.5;
        th   = theta_f * (first .* 2.*s.^2 + ~first .* (1 - 2*(1 - s).^2));
        thd  = in .* a*T .* (first .* s + ~first .* (1 - s));
        thdd = in .* a .* (first - ~first);
    case 'quintic'
        th   = theta_f * (10*s.^3 - 15*s.^4 + 6*s.^5);
        thd  = in .* theta_f/T .* (30*s.^2 - 60*s.^3 + 30*s.^4);
        thdd = in .* theta_f/T^2 .* (60*s - 180*s.^2 + 120*s.^3);
    otherwise
        error('unknown profile %s', type);
end
end
