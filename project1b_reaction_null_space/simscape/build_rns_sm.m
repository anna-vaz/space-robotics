function build_rns_sm(modelName)
%BUILD_RNS_SM  Simscape Multibody plant + Reaction Null Space controller (project 1b).
%
%   Plant (Simscape derives the dynamics; nothing from model.py is used):
%     World --[Planar Joint x0 y0 th0, free]--> base --> 3 revolute joints --> links 1, 2, 3
%   The joints are MOTION-driven: they follow the joint angles q_m(t) exactly and
%   Simscape works out the torques. The base is not actuated, so how it moves is
%   Simscape's answer to the joint motion (momentum conservation).
%
%   Controller (MATLAB Function, same law as path_tracking.py), evaluated on the
%   MEASURED base pose from Simscape:
%     v    = v_des + KP_EE (x_des - x_e)
%     qd_m = J*^+ v + s n,   n = r1 x r2,   s from METHOD (0 min-norm, 1 exact RNS, 2 priority)
%   q_m is the integral of qd_m; qdd_m is the time derivative of the control law
%   (central difference), as needed for motion actuation.

if nargin < 1, modelName = 'rns_sm'; end
if bdIsLoaded(modelName), close_system(modelName, 0); end
f = [modelName '.slx'];
if exist(f, 'file'), delete(f); end
new_system(modelName);
load_system('sm_lib'); load_system('nesl_utility'); load_system('simulink');
m = modelName;
port = @(b, side, k) getport(m, b, side, k);
P = @(a, ia, b, ib) add_line(m, port(a, 'RConn', ia), port(b, 'LConn', ib), 'autorouting', 'smart');
L = @(a, b) add_line(m, a, b, 'autorouting', 'smart');

% ---- environment ---------------------------------------------------------------
add_block('sm_lib/Frames and Transforms/World Frame', [m '/World'], 'Position', [40 120 80 160]);
add_block('sm_lib/Utilities/Mechanism Configuration', [m '/Mechanism Configuration'], ...
    'UniformGravity', 'None', 'Position', [40 30 80 70]);
add_block('nesl_utility/Solver Configuration', [m '/Solver Configuration'], 'Position', [40 210 80 250]);
add_line(m, port('Mechanism Configuration', 'RConn', 1), port('World', 'RConn', 1), 'autorouting', 'smart');
add_line(m, port('Solver Configuration', 'RConn', 1), port('World', 'RConn', 1), 'autorouting', 'smart');

% ---- base ----------------------------------------------------------------------
bj = 'Base joint (x0 y0 th0)';
add_block('sm_lib/Joints/Planar Joint', [m '/' bj], 'Position', [130 120 180 180]);
for ax = {'Px', 'Py', 'Rz'}
    set_param([m '/' bj], [ax{1} 'SensePosition'], 'on', [ax{1} 'SenseVelocity'], 'on');
end
P('World', 1, bj, 1);
add_block('sm_lib/Body Elements/Brick Solid', [m '/Base'], 'Position', [230 30 280 80]);
solid([m '/Base'], '[a0 a0 0.2]', 'm0', '[0.55*I0 0.55*I0 I0]', [0.6 0.6 0.7]);
add_line(m, port('Base', 'RConn', 1), port(bj, 'RConn', 1), 'autorouting', 'smart');
xform(m, 'Base to shoulder', '[b0(1) b0(2) 0]', [230 130 280 170]);
P(bj, 1, 'Base to shoulder', 1);

% ---- arm: joint k -> link k (CoM at half length) -> next joint (full length) ------
prev = 'Base to shoulder';
colors = {[0.85 0.45 0.2], [0.2 0.5 0.85], [0.3 0.7 0.3]};
for k = 1:3
    x = 330 + 260*(k-1);
    jn = sprintf('Joint %d (q%d)', k, k);
    add_block('sm_lib/Joints/Revolute Joint', [m '/' jn], 'Position', [x 120 x+50 180], ...
        'MotionActuationMode', 'InputMotion', 'TorqueActuationMode', 'ComputedTorque');
    P(prev, 1, jn, 1);
    xform(m, sprintf('Joint %d to link %d CoM', k, k), sprintf('[LEN(%d)/2 0 0]', k), [x+90 40 x+140 80]);
    P(jn, 1, sprintf('Joint %d to link %d CoM', k, k), 1);
    add_block('sm_lib/Body Elements/Brick Solid', [m sprintf('/Link %d', k)], 'Position', [x+180 40 x+230 80]);
    solid([m sprintf('/Link %d', k)], sprintf('[LEN(%d) 0.08 0.08]', k), sprintf('MS(%d)', k), ...
        sprintf('[0.01 IN(%d) IN(%d)]', k, k), colors{k});
    add_line(m, port(sprintf('Link %d', k), 'RConn', 1), port(sprintf('Joint %d to link %d CoM', k, k), 'RConn', 1), 'autorouting', 'smart');
    if k < 3
        nxt = sprintf('Joint %d to joint %d', k, k+1);
        xform(m, nxt, sprintf('[LEN(%d) 0 0]', k), [x+90 130 x+140 170]);
        P(jn, 1, nxt, 1);
        prev = nxt;
    end
    % motion input: q, qd, qdd from the controller
    cv = sprintf('SL-PS q%d', k);
    add_block('nesl_utility/Simulink-PS Converter', [m '/' cv], 'Unit', 'rad', ...
        'FilteringAndDerivatives', 'provide', 'UdotUserProvided', '2', 'Position', [x-40 230 x-10 290]);
    add_line(m, port(cv, 'RConn', 1), port(jn, 'LConn', 2), 'autorouting', 'smart');
    for d = 1:3
        add_block('simulink/Signal Routing/From', [m sprintf('/From q%d_%d', k, d)], ...
            'GotoTag', sprintf('qm_%d', d), 'ShowName', 'off', 'Position', [x-130 220+25*(d-1) x-90 236+25*(d-1)]);
        add_block('simulink/Signal Routing/Selector', [m sprintf('/sel q%d_%d', k, d)], ...
            'InputPortWidth', '3', 'Indices', num2str(k), 'ShowName', 'off', ...
            'Position', [x-80 220+25*(d-1) x-60 236+25*(d-1)]);
        L(sprintf('From q%d_%d/1', k, d), sprintf('sel q%d_%d/1', k, d));
        L(sprintf('sel q%d_%d/1', k, d), sprintf('%s/%d', cv, d));
    end
end

% ---- base sensing -> Goto ------------------------------------------------------
names = {'x0', 'dx0', 'y0', 'dy0', 'th0', 'dth0'};
units = {'m', 'm/s', 'm', 'm/s', 'rad', 'rad/s'};
for k = 1:6
    y = 330 + 40*(k-1);
    add_block('nesl_utility/PS-Simulink Converter', [m '/PS ' names{k}], 'Unit', units{k}, 'Position', [150 y 180 y+20]);
    add_block('simulink/Signal Routing/Goto', [m '/Goto ' names{k}], 'GotoTag', names{k}, ...
        'ShowName', 'off', 'Position', [210 y 260 y+20]);
    add_line(m, port(bj, 'RConn', 1+k), port(['PS ' names{k}], 'LConn', 1), 'autorouting', 'smart');
    L(['PS ' names{k} '/1'], ['Goto ' names{k} '/1']);
end

% ---- controller ------------------------------------------------------------------
add_block('simulink/Sources/Clock', [m '/Clock'], 'Position', [380 360 410 390]);
qb = {'x0', 'y0', 'th0'}; vb = {'dx0', 'dy0', 'dth0'};
add_block('simulink/Signal Routing/Mux', [m '/q_b'], 'Inputs', '3', 'Position', [460 410 465 470]);
add_block('simulink/Signal Routing/Mux', [m '/qd_b'], 'Inputs', '3', 'Position', [460 490 465 550]);
for k = 1:3
    add_block('simulink/Signal Routing/From', [m '/From ' qb{k}], 'GotoTag', qb{k}, 'ShowName', 'off', ...
        'Position', [390 405+20*(k-1) 430 419+20*(k-1)]);
    add_block('simulink/Signal Routing/From', [m '/From ' vb{k}], 'GotoTag', vb{k}, 'ShowName', 'off', ...
        'Position', [390 485+20*(k-1) 430 499+20*(k-1)]);
    L(['From ' qb{k} '/1'], sprintf('q_b/%d', k));
    L(['From ' vb{k} '/1'], sprintf('qd_b/%d', k));
end
add_block('simulink/Continuous/Integrator', [m '/integrate q_m'], 'InitialCondition', 'QM0(:)', ...
    'Position', [800 400 830 430]);
addFcn(m, 'RNS controller', [520 360 720 560], ctrlCode(), ...
    {'LEN', 'MS', 'IN', 'm0', 'I0', 'b0', 'KP_EE', 'K_TH', 'LAM', 'METHOD', 'XS', 'XG', 'T_MOVE'});
L('Clock/1', 'RNS controller/1');
L('q_b/1', 'RNS controller/2');
L('integrate q_m/1', 'RNS controller/3');
L('RNS controller/1', 'integrate q_m/1');
tags = {'integrate q_m/1', 'RNS controller/1', 'RNS controller/2'};
for d = 1:3
    add_block('simulink/Signal Routing/Goto', [m sprintf('/Goto qm_%d', d)], 'GotoTag', sprintf('qm_%d', d), ...
        'ShowName', 'off', 'Position', [880 380+50*(d-1) 930 400+50*(d-1)]);
    L(tags{d}, sprintf('Goto qm_%d/1', d));
end

% ---- logging [x0 y0 th0 q1 q2 q3 dx0 dy0 dth0 qd1 qd2 qd3 detA ex ey] ----------
add_block('simulink/Signal Routing/Mux', [m '/log mux'], 'Inputs', '5', 'Position', [1000 380 1005 560]);
L('q_b/1', 'log mux/1');
L('integrate q_m/1', 'log mux/2');
L('qd_b/1', 'log mux/3');
L('RNS controller/1', 'log mux/4');
L('RNS controller/3', 'log mux/5');
add_block('simulink/Sinks/To Workspace', [m '/log'], 'VariableName', 'Y_sm', ...
    'SaveFormat', 'Array', 'Position', [1050 455 1110 485]);
L('log mux/1', 'log/1');

% Simscape marks its outputs as depending directly on the motion inputs, so Simulink
% sees an algebraic loop through the controller and solves it at every step.
set_param(m, 'AlgebraicLoopMsg', 'none');
set_param(m, 'StopTime', 'T_END', 'Solver', 'ode45', 'RelTol', '1e-10', 'AbsTol', '1e-12', ...
    'MaxStep', '0.01', 'OutputOption', 'SpecifiedOutputTimes', 'OutputTimes', '0:0.01:T_END', ...
    'SaveTime', 'on', 'TimeSaveName', 't_sm', 'SaveOutput', 'off', 'ReturnWorkspaceOutputs', 'on');
save_system(m);
end

% =============================================================================
function h = getport(m, blkName, side, k)
ph = get_param([m '/' blkName], 'PortHandles');
h = ph.(side)(k);
end

function solid(blk, dims, mass, moments, color)
set_param(blk, 'BrickDimensions', dims, 'InertiaType', 'Custom', 'Mass', mass, ...
    'CenterOfMass', '[0 0 0]', 'MomentsOfInertia', moments, 'ProductsOfInertia', '[0 0 0]', ...
    'GraphicDiffuseColor', mat2str(color));
end

function xform(m, name, offset, pos)
add_block('sm_lib/Frames and Transforms/Rigid Transform', [m '/' name], 'Position', pos, ...
    'TranslationMethod', 'Cartesian', 'TranslationCartesianOffset', offset);
end

function addFcn(m, name, pos, code, params)
blk = [m '/' name];
add_block('simulink/User-Defined Functions/MATLAB Function', blk, 'Position', pos);
chart = sfroot().find('-isa', 'Stateflow.EMChart', 'Path', blk);
chart.Script = code;
for k = 1:numel(params)
    d = chart.find('-isa', 'Stateflow.Data', 'Name', params{k});
    d.Scope = 'Parameter';
end
end

function c = ctrlCode()
c = fileread(fullfile(fileparts(mfilename('fullpath')), 'rns_controller.m'));
end
