function [A, tk] = input_shaper(type, f_d, zeta)
%INPUT_SHAPER  Impulse sequence (amplitudes A, times tk) that cancels the vibration
%   of a mode with damped frequency f_d [Hz] and damping ratio zeta.
%   Convolving any command with it leaves (ideally) no residual vibration.
%
%   'ZV'  : 2 impulses, half a period long. Zero vibration at the design frequency.
%   'ZVD' : 3 impulses, one period. Also zero DERIVATIVE w.r.t. frequency there,
%           so it tolerates frequency errors much better.
%   'EI'  : 3 impulses, one period. "Extra-insensitive": allows 5 % vibration at the
%           design frequency in exchange for an even wider tolerance band.
%   (Singer & Seering 1990; Singhose, Seering & Singer 1994.)
Td = 1/f_d;
K  = exp(-zeta*pi/sqrt(1 - zeta^2));
switch upper(type)
    case 'NONE', A = 1;                        tk = 0;
    case 'ZV',   A = [1, K] / (1 + K);         tk = [0, Td/2];
    case 'ZVD',  A = [1, 2*K, K^2] / (1 + K)^2; tk = [0, Td/2, Td];
    case 'EI'
        V = 0.05;                               % tolerable vibration at f_d
        A = [(1 + V)/4, (1 - V)/2, (1 + V)/4];  tk = [0, Td/2, Td];   % undamped form
    otherwise, error('unknown shaper %s', type);
end
end
