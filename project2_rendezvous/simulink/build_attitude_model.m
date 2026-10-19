function build_attitude_model(modelName)
%BUILD_ATTITUDE_MODEL  Create the Simulink model of the chaser attitude loop.
%
%   Clock -> Reference -> Controller -> Saturation -> Dynamics -> 1/s -> omega
%                                                     Kinematics -> 1/s -> beta
%
%   Same equations as attitude.py (Schaub & Junkins conventions):
%     beta_dot  = 1/2 [B(beta)] omega                (Euler parameters, scalar first)
%     omega_dot = J^-1 ( -omega x J omega + u )      (Euler's rotational equations)
%     u = -K eps - P d_omega + J w_r_dot_B + omega x J omega   (Lyapunov tracking law)
%   Parameters J, K, P, U_MAX, n, beta0, omega0 are read from the base workspace.

if nargin < 1, modelName = 'attitude_control'; end
if bdIsLoaded(modelName), close_system(modelName, 0); end
new_system(modelName);
load_system('simulink');
m = modelName;

% ---- blocks ------------------------------------------------------------------
add_block('simulink/Sources/Clock', [m '/Clock'], 'Position', [30 140 60 170]);
addFcn(m, 'Reference', [110 115 250 195], refCode(), {'n'});
addFcn(m, 'Controller', [330 100 480 240], ctrlCode(), {'J', 'K', 'P'});
add_block('simulink/Discontinuities/Saturation', [m '/Torque limit'], ...
    'UpperLimit', 'U_MAX', 'LowerLimit', '-U_MAX', ...
    'Position', [530 155 580 185]);
addFcn(m, 'Rigid-body dynamics', [630 150 780 210], dynCode(), {'J'});
add_block('simulink/Continuous/Integrator', [m '/Integrate omega'], ...
    'InitialCondition', 'omega0(:)', 'Position', [830 165 860 195]);
addFcn(m, 'EP kinematics', [630 290 780 350], kinCode(), {});
add_block('simulink/Continuous/Integrator', [m '/Integrate beta'], ...
    'InitialCondition', 'beta0(:)', 'Position', [830 305 860 335]);
toWs(m, 'log beta', 'beta_sl', [930 305 990 335]);
toWs(m, 'log omega', 'omega_sl', [930 165 990 195]);
toWs(m, 'log u', 'u_sl', [630 60 690 90]);
toWs(m, 'log t', 't_sl', [110 220 170 250]);

% ---- wiring ------------------------------------------------------------------
L = @(a, b) add_line(m, a, b, 'autorouting', 'smart');
L('Clock/1', 'Reference/1');
L('Clock/1', 'log t/1');
L('Reference/1', 'Controller/3');      % RN
L('Reference/2', 'Controller/4');      % omega_r
L('Reference/3', 'Controller/5');      % omega_r_dot
L('Integrate beta/1', 'Controller/1');
L('Integrate omega/1', 'Controller/2');
L('Controller/1', 'Torque limit/1');
L('Torque limit/1', 'Rigid-body dynamics/2');
L('Torque limit/1', 'log u/1');
L('Integrate omega/1', 'Rigid-body dynamics/1');
L('Rigid-body dynamics/1', 'Integrate omega/1');
L('Integrate omega/1', 'EP kinematics/2');
L('Integrate beta/1', 'EP kinematics/1');
L('EP kinematics/1', 'Integrate beta/1');
L('Integrate beta/1', 'log beta/1');
L('Integrate omega/1', 'log omega/1');

% ---- solver: same accuracy as the Python run ------------------------------------
set_param(m, 'StopTime', '600', 'Solver', 'ode45', 'RelTol', '1e-11', 'AbsTol', '1e-12', ...
    'MaxStep', '1', 'OutputOption', 'SpecifiedOutputTimes', 'OutputTimes', '0:0.2:600', ...
    'SaveTime', 'off', 'SaveOutput', 'off', 'ReturnWorkspaceOutputs', 'on');
end

% =============================================================================
function addFcn(m, name, pos, code, params)
% MATLAB Function block with the given code; listed inputs become parameters.
blk = [m '/' name];
add_block('simulink/User-Defined Functions/MATLAB Function', blk, 'Position', pos);
chart = sfroot().find('-isa', 'Stateflow.EMChart', 'Path', blk);
chart.Script = code;
for k = 1:numel(params)
    d = chart.find('-isa', 'Stateflow.Data', 'Name', params{k});
    d.Scope = 'Parameter';
end
end

function toWs(m, name, var, pos)
add_block('simulink/Sinks/To Workspace', [m '/' name], 'VariableName', var, ...
    'SaveFormat', 'Array', 'Position', pos);
end

function c = refCode()
c = strjoin({
'function [RN, w_r, w_r_dot] = reference(t, n)'
'% Reference frame R at the V-bar hold point: r1 = along-track (towards the'
'% target), r3 = orbit normal. LVLH rotates at n about the orbit normal.'
'th = n*t;'
'r1 = [-sin(th); cos(th); 0];'
'r3 = [0; 0; 1];'
'r2 = cross(r3, r1);'
'RN = [r1.''; r2.''; r3.''];'
'w_r = [0; 0; n];'
'w_r_dot = [0; 0; 0];'
}, newline);
end

function c = dynCode()
c = strjoin({
'function omega_dot = dynamics(omega, u, J)'
'% Euler''s rotational equations of motion'
'omega_dot = J \ (-cross(omega, J*omega) + u);'
}, newline);
end

function c = kinCode()
c = strjoin({
'function beta_dot = kinematics(beta, omega)'
'% Euler-parameter kinematic differential equation'
'B = [-beta(2) -beta(3) -beta(4);'
'      beta(1) -beta(4)  beta(3);'
'      beta(4)  beta(1) -beta(2);'
'     -beta(3)  beta(2)  beta(1)];'
'beta_dot = 0.5*B*omega;'
}, newline);
end

function c = ctrlCode()
c = strjoin({
'function u = controller(beta, omega, RN, w_r, w_r_dot, J, K, P)'
'% Lyapunov tracking law (unsaturated; the limit is a separate block)'
'BN = ep2dcm(beta);'
'e_b = dcm2ep(BN*RN.'');                    % error EPs of B relative to R'
'w_rB = BN*w_r;'
'w_rdotB = BN*w_r_dot - cross(omega, w_rB); % derivative seen from B'
'dw = omega - w_rB;'
'u = -K*e_b(2:4) - P*dw + J*w_rdotB + cross(omega, J*omega);'
'end'
''
'function C = ep2dcm(b)'
'C = [b(1)^2+b(2)^2-b(3)^2-b(4)^2, 2*(b(2)*b(3)+b(1)*b(4)), 2*(b(2)*b(4)-b(1)*b(3));'
'     2*(b(2)*b(3)-b(1)*b(4)), b(1)^2-b(2)^2+b(3)^2-b(4)^2, 2*(b(3)*b(4)+b(1)*b(2));'
'     2*(b(2)*b(4)+b(1)*b(3)), 2*(b(3)*b(4)-b(1)*b(2)), b(1)^2-b(2)^2-b(3)^2+b(4)^2];'
'end'
''
'function b = dcm2ep(C)'
'% Sheppard''s method, short-rotation set (b0 >= 0)'
'tr = trace(C);'
'b2 = [(1+tr)/4; (1+2*C(1,1)-tr)/4; (1+2*C(2,2)-tr)/4; (1+2*C(3,3)-tr)/4];'
'[~, i] = max(b2);'
'b = zeros(4,1);'
'b(i) = sqrt(b2(i));'
'switch i'
'  case 1'
'    b(2) = (C(2,3)-C(3,2))/(4*b(1)); b(3) = (C(3,1)-C(1,3))/(4*b(1)); b(4) = (C(1,2)-C(2,1))/(4*b(1));'
'  case 2'
'    b(1) = (C(2,3)-C(3,2))/(4*b(2)); b(3) = (C(1,2)+C(2,1))/(4*b(2)); b(4) = (C(3,1)+C(1,3))/(4*b(2));'
'  case 3'
'    b(1) = (C(3,1)-C(1,3))/(4*b(3)); b(2) = (C(1,2)+C(2,1))/(4*b(3)); b(4) = (C(2,3)+C(3,2))/(4*b(3));'
'  otherwise'
'    b(1) = (C(1,2)-C(2,1))/(4*b(4)); b(2) = (C(3,1)+C(1,3))/(4*b(4)); b(3) = (C(2,3)+C(3,2))/(4*b(4));'
'end'
'if b(1) < 0, b = -b; end'
'end'
}, newline);
end
