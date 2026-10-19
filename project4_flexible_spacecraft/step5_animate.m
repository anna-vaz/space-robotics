%STEP5_ANIMATE  GIF: the same 90-degree slew with three strategies, on a
%   spacecraft whose panels are 40 % softer than the designers assumed.
%   Panel bending is exaggerated 80x so that millimetres are visible.
clear; close all
here = fileparts(mfilename('fullpath'));
p = flex_params();  p.EI = 0.6*p.EI;          % the real panels: softer than designed for
m = flex_model(p);  J = p.J_total;  n = m.n;
S2 = load(fullfile(here, 'results_step2.mat'));
S3 = load(fullfile(here, 'results_step3.mat'));
S4 = load(fullfile(here, 'results_step4.mat'));
theta_f = S2.theta_f;  T = S2.T_bb;
[Az, tz] = input_shaper('ZVD', S3.f_d, S3.z_d);
c0 = S2.ctrl;  c4 = c0;  c4.K2 = S4.Km;
cases = {'PD (rigid design)', c0, 1, 0;  'PD + ZVD input shaper', c0, Az, tz;  'PD + robust H_\infty', c4, 1, 0};
t_end = 80;  tf = 0:0.4:t_end;
sims = cell(3,1);
for k = 1:3
    A = cases{k,3};  tk = cases{k,4};
    ref = @(t) shaped(t, A, tk, theta_f, T, J);
    o = flex_simulate(m, p, cases{k,2}, ref, t_end, unique([tk, tk + T/2, tk + T]));
    [tu, iu] = unique(o.t);
    sims{k} = struct('x', interp1(tu, o.x(iu,:), tf), 'err', interp1(tu, o.theta(iu) - theta_f, tf), ...
                     'tip', interp1(tu, o.tip(iu), tf));
end

xs = linspace(0, p.L, 40);
Phi = zeros(numel(xs), n);
for i = 1:n, Phi(:,i) = m.phi{i}(xs); end
amp = 80;                                       % exaggeration of the bending
gif = fullfile(here, 'figures', 'flexible_slew.gif');
f = figure('Position', [50 50 1200 470], 'Color', 'w'); theme(f, 'light');
cols = [0 0 0; 0 0.447 0.741; 0.929 0.694 0.125];
for j = 1:numel(tf)
    clf(f);
    for k = 1:3
        ax = subplot(1,3,k); hold(ax, 'on'); axis(ax, 'equal'); axis(ax, [-4.2 4.2 -5.3 4.3]); axis(ax, 'off');
        x = sims{k}.x(j,:);  th = x(1);  e1 = x(2:1+n)';  e2 = x(2+n:1+2*n)';
        R = [cos(th) -sin(th); sin(th) cos(th)];
        plot(ax, 4*cos(linspace(0, pi/2, 50)), 4*sin(linspace(0, pi/2, 50)), ':', 'Color', [0.7 0.7 0.7]);
        plot(ax, [0 0], [0 4.1], '--', 'Color', [0.5 0.5 0.5]);           % target direction
        hub = R*[p.r -p.r -p.r p.r p.r; p.r p.r -p.r -p.r p.r];
        fill(ax, hub(1,:), hub(2,:), [0.85 0.85 0.85], 'EdgeColor', 'k');
        w1 = amp*Phi*e1;  w2 = amp*Phi*e2;
        P1 = R*[p.r + xs; w1'];  P2 = R*[-(p.r + xs); -w2'];
        plot(ax, P1(1,:), P1(2,:), '-', 'Color', cols(k,:), 'LineWidth', 4);
        plot(ax, P2(1,:), P2(2,:), '-', 'Color', cols(k,:), 'LineWidth', 4);
        title(ax, cases{k,1}, 'FontSize', 12);
        txt = sprintf('angle to target %8.4f deg\npanel tip       %6.2f mm', -sims{k}.err(j)*180/pi, 1e3*sims{k}.tip(j));
        text(ax, -2.6, -4.7, txt, 'FontName', 'Consolas', 'FontSize', 10);
    end
    annotation(f, 'textbox', [0.1 0.01 0.8 0.06], 'String', sprintf(['t = %5.1f s   (slew command ends at %.1f s)' ...
        '   panels 40%% softer than designed; bending x%d'], tf(j), T, amp), 'EdgeColor', 'none', ...
        'HorizontalAlignment', 'center', 'FontSize', 10, 'FitBoxToText', 'off');
    img = print(f, '-RGBImage', '-r85');
    [I, map] = rgb2ind(img, 64, 'nodither');
    if j == 1
        imwrite(I, map, gif, 'gif', 'LoopCount', Inf, 'DelayTime', 0.08);
    else
        imwrite(I, map, gif, 'gif', 'WriteMode', 'append', 'DelayTime', 0.08);
    end
end
fprintf('saved %s (%d frames)\n', gif, numel(tf));

function r = shaped(t, A, tk, theta_f, T, J)
r = zeros(3, numel(t));
for i = 1:numel(A)
    [th, thd, thdd] = slew_profile('bangbang', t - tk(i), theta_f, T);
    r = r + A(i)*[th; thd; J*thdd];
end
end
