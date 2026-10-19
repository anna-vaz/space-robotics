function mdl = build_flex_simulink(model_dir, P)
%BUILD_FLEX_SIMULINK  Script-generated Simulink model of the closed-loop slew.
%
%   Commanded acceleration (three Step blocks: +a at 0, -2a at T/2, +a at T)
%     -> two Integrators give theta_r and theta_r_dot, a Gain gives u_ff = J a_r
%   errors y = [theta_r - theta; theta_r_dot - theta_dot]  (Selector picks theta, theta_dot)
%   u = sat( [Kp Kd] y + K2(y) + u_ff )       (State-Space block for K2)
%   plant: State-Space block, outputs [theta; theta_dot; tip]
%
%   P: struct with A,B,C,D (plant), Ak,Bk,Ck,Dk (K2), Kp, Kd, J, a, T, umax, t_out
%   Variables are written into the model workspace, so the .slx is self-contained.
mdl = 'flex_slew';
f = fullfile(model_dir, [mdl '.slx']);
if bdIsLoaded(mdl), close_system(mdl, 0); end
if exist(f, 'file'), delete(f); end
new_system(mdl);  load_system(mdl);
mw = get_param(mdl, 'ModelWorkspace');
for n = fieldnames(P)'
    assignin(mw, n{1}, P.(n{1}));
end
set_param(mdl, 'Solver', 'ode45', 'RelTol', '1e-10', 'AbsTol', '1e-12', 'StopTime', 't_out(end)', ...
          'OutputOption', 'SpecifiedOutputTimes', 'OutputTimes', 't_out', ...
          'SaveOutput', 'off', 'SaveTime', 'off', 'SaveState', 'off', 'ReturnWorkspaceOutputs', 'on');

add = @(lib, name, pos, varargin) add_block(lib, [mdl '/' name], 'Position', pos, varargin{:});
% --- reference generator ---
add('simulink/Sources/Step', 'acc +a',   [30 30 60 60],   'Time', '0',   'Before', '0', 'After', 'a');
add('simulink/Sources/Step', 'acc -2a',  [30 90 60 120],  'Time', 'T/2', 'Before', '0', 'After', '-2*a');
add('simulink/Sources/Step', 'acc +a end', [30 150 60 180], 'Time', 'T', 'Before', '0', 'After', 'a');
add('simulink/Math Operations/Sum', 'a_r', [100 80 120 130], 'Inputs', '+++', 'IconShape', 'rectangular');
add('simulink/Continuous/Integrator', 'int1', [160 90 190 120]);
add('simulink/Continuous/Integrator', 'int2', [230 90 260 120]);
add('simulink/Math Operations/Gain', 'J (feedforward)', [160 160 210 190], 'Gain', 'J');
% --- errors and control law ---
add('simulink/Signal Routing/Mux', 'ref', [300 80 305 140], 'Inputs', '2');
add('simulink/Signal Routing/Selector', 'angle & rate', [560 230 620 260], 'InputPortWidth', '3', ...
    'Indices', '[1 2]', 'Orientation', 'left');
add('simulink/Math Operations/Sum', 'error y', [370 100 390 140], 'Inputs', '+-', 'IconShape', 'rectangular');
add('simulink/Math Operations/Gain', 'PD [Kp Kd]', [430 70 490 110], 'Gain', '[Kp Kd]', 'Multiplication', 'Matrix(K*u)');
add('simulink/Continuous/State-Space', 'K2 robust', [430 130 490 170], 'A', 'Ak', 'B', 'Bk', 'C', 'Ck', 'D', 'Dk');
add('simulink/Math Operations/Sum', 'u total', [530 100 550 180], 'Inputs', '+++', 'IconShape', 'rectangular');
add('simulink/Discontinuities/Saturation', 'torque limit', [580 125 620 155], 'UpperLimit', 'umax', 'LowerLimit', '-umax');
add('simulink/Continuous/State-Space', 'spacecraft', [650 120 730 160], 'A', 'A', 'B', 'B', 'C', 'C', 'D', 'D');
add('simulink/Sinks/To Workspace', 'y_out', [800 120 860 150], 'VariableName', 'y_sl', 'SaveFormat', 'Array');
add('simulink/Sinks/To Workspace', 'u_out', [650 40 710 70], 'VariableName', 'u_sl', 'SaveFormat', 'Array');

c = @(a, b) add_line(mdl, a, b, 'autorouting', 'smart');
c('acc +a/1', 'a_r/1');  c('acc -2a/1', 'a_r/2');  c('acc +a end/1', 'a_r/3');
c('a_r/1', 'int1/1');  c('int1/1', 'int2/1');  c('a_r/1', 'J (feedforward)/1');
c('int2/1', 'ref/1');  c('int1/1', 'ref/2');
c('ref/1', 'error y/1');  c('angle & rate/1', 'error y/2');
c('error y/1', 'PD [Kp Kd]/1');  c('error y/1', 'K2 robust/1');
c('PD [Kp Kd]/1', 'u total/1');  c('K2 robust/1', 'u total/2');  c('J (feedforward)/1', 'u total/3');
c('u total/1', 'torque limit/1');  c('torque limit/1', 'spacecraft/1');  c('torque limit/1', 'u_out/1');
c('spacecraft/1', 'y_out/1');  c('spacecraft/1', 'angle & rate/1');
save_system(mdl, f);
end
