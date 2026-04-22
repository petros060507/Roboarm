function workspace_analysis(robot, joint_a, joint_b, N_pts)
% WORKSPACE_ANALYSIS  Тепловые карты τ_req при сканировании двух суставов.
%
%   workspace_analysis(robot)
%     — сканирует J2 vs J3 (наиболее нагруженная пара), 40×40 точек
%
%   workspace_analysis(robot, joint_a, joint_b, N_pts)
%     — пользовательский выбор пары суставов и разрешения сетки
%
%   Для каждой точки сетки (q_a, q_b) остальные суставы = 0.
%   Строятся:
%     • тепловая карта max|τ_req| по всем суставам
%     • тепловая карта max коэф. использования привода
%     • 6 subplots для каждого сустава отдельно

    if nargin < 2; joint_a = 2; end
    if nargin < 3; joint_b = 3; end
    if nargin < 4; N_pts   = 40; end

    n = robot.n;
    q_range_a = linspace(-pi/2, pi/2, N_pts);
    q_range_b = linspace(-pi/2, pi/2, N_pts);

    tau_map  = zeros(N_pts, N_pts, n);   % [Н·м]
    util_map = zeros(N_pts, N_pts, n);   % коэф. использования

    q_base = zeros(1, n);

    fprintf('\n  Workspace analysis: J%d vs J%d  [%dx%d] ...', ...
            joint_a, joint_b, N_pts, N_pts);

    for ia = 1:N_pts
        for ib = 1:N_pts
            q = q_base;
            q(joint_a) = q_range_a(ia);
            q(joint_b) = q_range_b(ib);
            [~, tau_req, util] = static_torque(robot, q);
            tau_map(ia, ib, :)  = tau_req;
            util_map(ia, ib, :) = util;
        end
    end

    fprintf(' готово.\n');

    q_a_deg = rad2deg(q_range_a);
    q_b_deg = rad2deg(q_range_b);

    max_tau  = max(abs(tau_map),  [], 3);   % [N_pts x N_pts]
    max_util = max(util_map, [], 3);

    % ── Фигура 1: общие карты ─────────────────────────────────────────────
    fig1 = figure('Name','Workspace Analysis — Summary', ...
                  'Color','#1a1a2e', 'Position',[50 50 1100 480]);

    % -- max |τ_req| --
    ax1 = subplot(1,2,1, 'Parent', fig1);
    imagesc(ax1, q_b_deg, q_a_deg, max_tau);
    set_dark_axes(ax1);
    colormap(ax1, turbo);
    cb1 = colorbar(ax1); cb1.Color = 'w'; cb1.Label.String = 'Н·м';
    xlabel(ax1, sprintf('q%d [°]', joint_b), 'Color','w');
    ylabel(ax1, sprintf('q%d [°]', joint_a), 'Color','w');
    title(ax1, 'max |τ_{req}| по суставам', 'Color','w','FontSize',12,'FontWeight','bold');
    axis(ax1,'xy');
    add_contour_overlay(ax1, q_b_deg, q_a_deg, max_tau);

    % -- max utilization --
    ax2 = subplot(1,2,2, 'Parent', fig1);
    imagesc(ax2, q_b_deg, q_a_deg, max_util);
    set_dark_axes(ax2);
    cmap2 = make_utilization_cmap();
    colormap(ax2, cmap2);
    clim(ax2, [0 1.5]);
    cb2 = colorbar(ax2); cb2.Color = 'w';
    cb2.Label.String = 'util (>1 = перегруз)';
    xlabel(ax2, sprintf('q%d [°]', joint_b), 'Color','w');
    ylabel(ax2, sprintf('q%d [°]', joint_a), 'Color','w');
    title(ax2, 'max коэф. использования привода', 'Color','w','FontSize',12,'FontWeight','bold');
    axis(ax2,'xy');
    % линия util = 1
    hold(ax2,'on');
    contour(ax2, q_b_deg, q_a_deg, max_util, [1 1], 'w--', 'LineWidth', 1.5);
    hold(ax2,'off');

    sgtitle(fig1, sprintf('Рабочее пространство: J%d vs J%d', joint_a, joint_b), ...
            'Color','w','FontSize',14,'FontWeight','bold','BackgroundColor','#1a1a2e');

    % ── Фигура 2: τ_req для каждого сустава ──────────────────────────────
    fig2 = figure('Name','Workspace Analysis — Per Joint', ...
                  'Color','#1a1a2e', 'Position',[50 560 1300 500]);

    cmap_per = cool(256);
    for ji = 1:n
        ax = subplot(2, 3, ji, 'Parent', fig2);
        data = squeeze(tau_map(:,:,ji));
        imagesc(ax, q_b_deg, q_a_deg, data);
        set_dark_axes(ax);
        colormap(ax, cmap_per);
        cb = colorbar(ax); cb.Color = 'w';
        xlabel(ax, sprintf('q%d [°]', joint_b), 'Color','w','FontSize',8);
        ylabel(ax, sprintf('q%d [°]', joint_a), 'Color','w','FontSize',8);
        title(ax, sprintf('J%d  τ_{req} [Н·м]  max=%.1f', ji, max(abs(data(:)))), ...
              'Color','w','FontSize',10,'FontWeight','bold');
        axis(ax,'xy');
    end

    sgtitle(fig2, 'τ_{req} по каждому суставу', ...
            'Color','w','FontSize',13,'FontWeight','bold','BackgroundColor','#1a1a2e');
end


% ─────────────────────────────────────────────────────────────────────────
function set_dark_axes(ax)
    set(ax, 'Color','#16213e', 'XColor','w', 'YColor','w', ...
            'GridColor','#0f3460', 'FontSize',9);
end

function add_contour_overlay(ax, xv, yv, data)
    hold(ax,'on');
    [~,h] = contour(ax, xv, yv, data, 5, 'LineWidth', 0.6);
    h.LineColor = [0.7 0.7 0.7];
    hold(ax,'off');
end

function cmap = make_utilization_cmap()
% Зелёный 0→0.8, жёлтый 0.8→1.0, красный 1.0→1.5
    n1 = 128; n2 = 32; n3 = 96;
    g1 = [linspace(0.1,0.8,n1)', linspace(0.7,0.9,n1)', linspace(0.1,0.2,n1)'];
    g2 = [linspace(0.8,1.0,n2)', linspace(0.9,0.85,n2)', linspace(0.2,0.1,n2)'];
    g3 = [ones(n3,1), linspace(0.85,0.1,n3)', linspace(0.1,0.1,n3)'];
    cmap = [g1; g2; g3];
    cmap = min(max(cmap,0),1);
end
