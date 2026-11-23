function build_free_floating_sm(modelName, fixedBase)
%BUILD_FREE_FLOATING_SM  Simscape Multibody model of the project-1 robot.
%
%   The robot is assembled from bodies, joints and frames; Simscape Multibody
%   derives the equations of motion itself. Nothing from dynamics.py is used,
%   so the model is an independent check of the hand-derived Lagrange equations.
%
%   World --[Planar Joint: x0, y0, th0]--> base (100 kg square plate)
%        --[+0.5 m]--> shoulder --[Revolute q1]--> link 1 (1 m, 10 kg)
%        --[+1 m]  --> elbow    --[Revolute q2]--> link 2 (1 m, 10 kg)
%
%   No gravity, no base actuation (free-floating). The two joints get the
%   same quintic reference and PD gains as simulation.py.
%   Parameters are read from the base workspace (see run_simscape_crosscheck.m).
%
%   fixedBase = true builds the SAME arm bolted to the ground instead: the Planar
%   Joint is removed and the base frame is the world frame (x0 = y0 = th0 = 0).

if nargin < 1, modelName = 'free_floating_sm'; end
if nargin < 2, fixedBase = false; end
if bdIsLoaded(modelName), close_system(modelName, 0); end
f = [modelName '.slx'];
if exist(f, 'file'), delete(f); end
new_system(modelName);
load_system('sm_lib'); load_system('nesl_utility'); load_system('simulink');
m = modelName;
port = @(b, side, k) getport(m, b, side, k);

% ---- environment ---------------------------------------------------------------
add_block('sm_lib/Frames and Transforms/World Frame', [m '/World'], 'Position', [40 300 80 340]);
add_block('sm_lib/Utilities/Mechanism Configuration', [m '/Mechanism Configuration'], ...
    'UniformGravity', 'None', 'Position', [40 200 80 240]);
add_block('nesl_utility/Solver Configuration', [m '/Solver Configuration'], 'Position', [40 400 80 440]);

% ---- base: planar joint (x0, y0, th0), all three unactuated and sensed ----------
if fixedBase
    bjName = 'World';                       % base frame = world frame
else
    bjName = 'Base joint (x0 y0 th0)';
    add_block('sm_lib/Joints/Planar Joint', [m '/' bjName], 'Position', [150 300 200 360]);
    for ax = {'Px', 'Py', 'Rz'}
        set_param([m '/' bjName], [ax{1} 'SensePosition'], 'on', [ax{1} 'SenseVelocity'], 'on');
    end
end
add_block('sm_lib/Body Elements/Brick Solid', [m '/Base'], 'Position', [280 220 330 270]);
solid([m '/Base'], '[a0 a0 0.2]', 'm0', '[0.55*I0 0.55*I0 I0]', [0.6 0.6 0.7]);

% ---- arm ------------------------------------------------------------------------
xform(m, 'Base to shoulder', '[b0(1) b0(2) 0]', [280 310 330 350]);
revolute(m, 'Joint 1 (q1)', [380 300 430 360]);
xform(m, 'Joint 1 to link 1 CoM', '[l1/2 0 0]', [480 220 530 260]);
add_block('sm_lib/Body Elements/Brick Solid', [m '/Link 1'], 'Position', [580 220 630 260]);
solid([m '/Link 1'], '[l1 0.08 0.08]', 'm1', '[0.01 I1 I1]', [0.85 0.45 0.2]);
xform(m, 'Joint 1 to elbow', '[l1 0 0]', [480 310 530 350]);
revolute(m, 'Joint 2 (q2)', [580 300 630 360]);
xform(m, 'Joint 2 to link 2 CoM', '[l2/2 0 0]', [680 220 730 260]);
add_block('sm_lib/Body Elements/Brick Solid', [m '/Link 2'], 'Position', [780 220 830 260]);
solid([m '/Link 2'], '[l2 0.08 0.08]', 'm2', '[0.01 I2 I2]', [0.2 0.5 0.85]);

% ---- frame connections (B -> F) -----------------------------------------------
C = @(a, ia, b, ib) add_line(m, port(a, 'RConn', ia), port(b, 'LConn', ib), 'autorouting', 'smart');
if ~fixedBase, C('World', 1, bjName, 1); end
add_line(m, port('Mechanism Configuration', 'RConn', 1), port('World', 'RConn', 1), 'autorouting', 'smart');
add_line(m, port('Solver Configuration', 'RConn', 1), port('World', 'RConn', 1), 'autorouting', 'smart');
add_line(m, port('Base', 'RConn', 1), port(bjName, 'RConn', 1), 'autorouting', 'smart');
C(bjName, 1, 'Base to shoulder', 1);
C('Base to shoulder', 1, 'Joint 1 (q1)', 1);
C('Joint 1 (q1)', 1, 'Joint 1 to link 1 CoM', 1);
add_line(m, port('Link 1', 'RConn', 1), port('Joint 1 to link 1 CoM', 'RConn', 1), 'autorouting', 'smart');
C('Joint 1 (q1)', 1, 'Joint 1 to elbow', 1);
C('Joint 1 to elbow', 1, 'Joint 2 (q2)', 1);
C('Joint 2 (q2)', 1, 'Joint 2 to link 2 CoM', 1);
add_line(m, port('Link 2', 'RConn', 1), port('Joint 2 to link 2 CoM', 'RConn', 1), 'autorouting', 'smart');

% ---- sensing: physical signals -> Simulink, sent on with Goto tags ---------------
% Planar joint sensor ports (after F): px, vx, py, vy, qz, wz
names = {'x0', 'dx0', 'y0', 'dy0', 'th0', 'dth0'};
units = {'m', 'm/s', 'm', 'm/s', 'rad', 'rad/s'};
for k = 1:6
    y = 420 + 45*(k-1);
    gotoTag(m, names{k}, [220 y 270 y+20]);
    if fixedBase                            % base does not move: log zeros
        add_block('simulink/Sources/Constant', [m '/zero ' names{k}], 'Value', '0', 'Position', [150 y 180 y+20]);
        add_line(m, ['zero ' names{k} '/1'], ['Goto ' names{k} '/1'], 'autorouting', 'smart');
    else
        ps2sl(m, ['PS ' names{k}], units{k}, [150 y 180 y+20]);
        add_line(m, port(bjName, 'RConn', 1+k), port(['PS ' names{k}], 'LConn', 1), 'autorouting', 'smart');
        add_line(m, ['PS ' names{k} '/1'], ['Goto ' names{k} '/1'], 'autorouting', 'smart');
    end
end
jn = {'Joint 1 (q1)', 'Joint 2 (q2)'};
for j = 1:2
    x = 400 + 200*(j-1);
    for v = {{'q', 'rad', 0}, {'dq', 'rad/s', 45}}
        nm = sprintf('%s%d', v{1}{1}, j);
        y = 420 + v{1}{3};
        ps2sl(m, ['PS ' nm], v{1}{2}, [x y x+30 y+20]);
        gotoTag(m, nm, [x+60 y x+110 y+20]);
        add_line(m, port(jn{j}, 'RConn', 2 + (v{1}{3} > 0)), port(['PS ' nm], 'LConn', 1), 'autorouting', 'smart');
        add_line(m, ['PS ' nm '/1'], ['Goto ' nm '/1'], 'autorouting', 'smart');
    end
end

% ---- controller: same quintic reference + joint PD as simulation.py --------------
L = @(a, b) add_line(m, a, b, 'autorouting', 'smart');
add_block('simulink/Sources/Clock', [m '/Clock'], 'Position', [420 560 450 590]);
fromTag(m, 'q1', 'ctrl', [400 610 450 630]);  fromTag(m, 'q2', 'ctrl', [400 640 450 660]);
fromTag(m, 'dq1', 'ctrl', [400 680 450 700]); fromTag(m, 'dq2', 'ctrl', [400 710 450 730]);
add_block('simulink/Signal Routing/Mux', [m '/q_m'], 'Inputs', '2', 'Position', [490 610 495 660]);
add_block('simulink/Signal Routing/Mux', [m '/dq_m'], 'Inputs', '2', 'Position', [490 680 495 730]);
addFcn(m, 'Joint PD + quintic reference', [540 560 720 730], pdCode(), {'KP', 'KD', 'Q_START', 'Q_GOAL', 'T_MOVE'});
add_block('simulink/Signal Routing/Demux', [m '/tau split'], 'Outputs', '2', 'Position', [760 600 765 700]);
L('From q1 ctrl/1', 'q_m/1');  L('From q2 ctrl/1', 'q_m/2');
L('From dq1 ctrl/1', 'dq_m/1'); L('From dq2 ctrl/1', 'dq_m/2');
L('Clock/1', 'Joint PD + quintic reference/1');
L('q_m/1', 'Joint PD + quintic reference/2');
L('dq_m/1', 'Joint PD + quintic reference/3');
L('Joint PD + quintic reference/1', 'tau split/1');
for j = 1:2
    blk = sprintf('SL-PS tau%d', j);
    y = 590 + 80*(j-1);
    add_block('nesl_utility/Simulink-PS Converter', [m '/' blk], 'Unit', 'N*m', 'Position', [820 y 850 y+20]);
    gotoTag(m, sprintf('tau%d', j), [820 y+35 870 y+55]);
    L(sprintf('tau split/%d', j), [blk '/1']);
    L(sprintf('tau split/%d', j), sprintf('Goto tau%d/1', j));
    add_line(m, port(blk, 'RConn', 1), port(jn{j}, 'LConn', 2), 'autorouting', 'smart');
end

% ---- logging: one vector [x0 y0 th0 q1 q2 dx0 dy0 dth0 dq1 dq2 tau1 tau2] -------
order = {'x0', 'y0', 'th0', 'q1', 'q2', 'dx0', 'dy0', 'dth0', 'dq1', 'dq2', 'tau1', 'tau2'};
add_block('simulink/Signal Routing/Mux', [m '/log mux'], 'Inputs', '12', 'Position', [1080 400 1085 760]);
for k = 1:12
    y = 405 + 30*(k-1);
    fromTag(m, order{k}, 'log', [990 y 1040 y+20]);
    L(sprintf('From %s log/1', order{k}), sprintf('log mux/%d', k));
end
add_block('simulink/Sinks/To Workspace', [m '/log'], 'VariableName', 'Y_sm', ...
    'SaveFormat', 'Array', 'Position', [1120 565 1180 595]);
L('log mux/1', 'log/1');

% ---- solver: same accuracy and output grid as the Python run -------------------
set_param(m, 'StopTime', 'T_END', 'Solver', 'ode45', 'RelTol', '1e-10', 'AbsTol', '1e-12', ...
    'MaxStep', '0.01', 'OutputOption', 'SpecifiedOutputTimes', 'OutputTimes', '0:0.01:T_END', ...
    'SaveTime', 'on', 'TimeSaveName', 't_sm', 'SaveOutput', 'off', 'ReturnWorkspaceOutputs', 'on', ...
    'SimscapeLogType', 'all', 'SimscapeLogName', 'simlog');
save_system(m);
end

% =============================================================================
function h = getport(m, blkName, side, k)
% port handle of a block (side = 'LConn' / 'RConn')
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

function revolute(m, name, pos)
add_block('sm_lib/Joints/Revolute Joint', [m '/' name], 'Position', pos, ...
    'TorqueActuationMode', 'InputTorque', 'SensePosition', 'on', 'SenseVelocity', 'on');
end

function gotoTag(m, tag, pos)
add_block('simulink/Signal Routing/Goto', [m '/Goto ' tag], 'GotoTag', tag, 'Position', pos, 'ShowName', 'off');
end

function fromTag(m, tag, suffix, pos)
add_block('simulink/Signal Routing/From', [m '/From ' tag ' ' suffix], 'GotoTag', tag, 'Position', pos, 'ShowName', 'off');
end

function ps2sl(m, name, unit, pos)
add_block('nesl_utility/PS-Simulink Converter', [m '/' name], 'Unit', unit, 'Position', pos);
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

function c = pdCode()
c = strjoin({
'function tau = joint_pd(t, q, qd, KP, KD, Q_START, Q_GOAL, T_MOVE)'
'% Quintic joint reference s = 10x^3 - 15x^4 + 6x^5 (zero vel/acc at both ends)'
'% and joint-space PD, exactly as simulation.py. The base is not controlled.'
'x = min(max(t/T_MOVE, 0), 1);'
's = 10*x^3 - 15*x^4 + 6*x^5;'
'ds = 0;'
'if t < T_MOVE, ds = (30*x^2 - 60*x^3 + 30*x^4)/T_MOVE; end'
'q_des = Q_START(:) + s*(Q_GOAL(:) - Q_START(:));'
'qd_des = ds*(Q_GOAL(:) - Q_START(:));'
'tau = KP(:).*(q_des - q(:)) + KD(:).*(qd_des - qd(:));'
}, newline);
end
