function visualize(robot, q, tau_req, tau_static)
% VISUALIZE  Трёхпанельная визуализация позы и статических моментов.
%
%   visualize(robot, q, tau_req)
%   visualize(robot, q, tau_req, tau_static)
%
%   Панели:
%     Левая  — 3D поза: звенья, суставы, CoM, оси z_i,
%                       круговые стрелки моментов, вектор g
%     Правая — Bar-chart τ_req с индикатором нагрузки привода

    if nargin < 4
        tau_static = tau_req / robot.k_dyn;   % восстанавливаем из tau_req
    end

    n   = robot.n;
    DH  = robot.DH;
    T   = forward_kinematics(DH, q);

    % ── Геометрия в мировой СК ───────────────────────────────────────────
    joints = zeros(3, n+1);    % начала суставов + TCP
    com_w  = zeros(3, n);
    z_axes = zeros(3, n);
    o_axes = zeros(3, n);

    for i = 1:n
        joints(:, i)   = T{i}(1:3, 4);
        joints(:, i+1) = T{i+1}(1:3, 4);
        com_w(:, i)    = T{i+1}(1:3, 4) + T{i+1}(1:3,1:3) * robot.com(i,:)';
        z_axes(:, i)   = T{i}(1:3, 3);
        o_axes(:, i)   = T{i}(1:3, 4);
    end

    % CoM нагрузки
    com_payload = T{n+1}(1:3,4) + T{n+1}(1:3,1:3) * robot.payload_com(:);

    % ── Масштаб ──────────────────────────────────────────────────────────
    all_pts    = [joints, com_w, com_payload];
    arm_reach  = max(vecnorm(all_pts - joints(:,1)));
    if arm_reach < 1e-4; arm_reach = 1; end

    tau_max    = max(abs(tau_req));
    if tau_max  < 1e-6; tau_max = 1; end
    arrow_sc   = 0.20 * arm_reach / tau_max;   % масштаб круговых стрелок
    link_r     = 0.013 * arm_reach;

    cmap       = cool(n);    % цвет по суставу

    % ═════════════════════════════════════════════════════════════════════
    fig = figure('Name','Robot Arm — Static Torque Visualization', ...
                 'Color','#1a1a2e', 'Position',[60 40 1300 700]);

    % ── ЛЕВАЯ ПАНЕЛЬ: 3D ─────────────────────────────────────────────────
    ax3 = subplot('Position',[0.03 0.06 0.54 0.86], 'Parent', fig);
    hold(ax3,'on'); grid(ax3,'on'); box(ax3,'on');
    set(ax3, 'Color','#16213e', 'GridColor','#0f3460', 'GridAlpha',0.45, ...
             'XColor','w', 'YColor','w', 'ZColor','w', ...
             'FontSize',9, 'LineWidth',0.5);
    axis(ax3,'equal');
    xlabel(ax3,'X [м]','Color','w'); ylabel(ax3,'Y [м]','Color','w');
    zlabel(ax3,'Z [м]','Color','w');
    title(ax3,'Поза роборуки  (статические моменты)', ...
          'Color','w','FontSize',12,'FontWeight','bold');
    view(ax3, 38, 22);

    % -- Базовая плита -------------------------------------------------------
    draw_base_plate(ax3, joints(:,1), arm_reach);

    % -- Звенья (трубки) -----------------------------------------------------
    for i = 1:n
        draw_tube(ax3, joints(:,i), joints(:,i+1), link_r, cmap(i,:));
    end

    % -- Суставы (сферы) -----------------------------------------------------
    for i = 1:n+1
        if     i == 1;   col = '#f5a623';   r_s = 2.2 * link_r;  % база
        elseif i == n+1; col = '#7ed321';   r_s = 2.0 * link_r;  % TCP
        else;            col = '#cccccc';   r_s = 1.7 * link_r;
        end
        draw_sphere(ax3, joints(:,i), r_s, col);
    end

    % -- CoM звеньев ---------------------------------------------------------
    for i = 1:n
        draw_sphere(ax3, com_w(:,i), 1.0*link_r, '#ff6b6b');
        mid = (joints(:,i) + joints(:,i+1)) / 2;
        plot3(ax3, [com_w(1,i) mid(1)], [com_w(2,i) mid(2)], [com_w(3,i) mid(3)], ...
              '--', 'Color',[1 0.42 0.42 0.6], 'LineWidth',0.9);
    end

    % -- CoM нагрузки --------------------------------------------------------
    draw_sphere(ax3, com_payload, 1.2*link_r, '#ff9f43');
    plot3(ax3, [com_payload(1) joints(1,n+1)], ...
               [com_payload(2) joints(2,n+1)], ...
               [com_payload(3) joints(3,n+1)], ...
          '--', 'Color',[1 0.62 0.26 0.6], 'LineWidth',0.9);

    % -- Оси вращения z_i и круговые стрелки моментов ----------------------
    for i = 1:n
        z   = z_axes(:,i);
        org = o_axes(:,i);
        len_ax = 0.11 * arm_reach;

        % ось z_i (белая)
        draw_arrow3d(ax3, org, org + len_ax*z, 0.003*arm_reach, [1 1 1], 0.50);

        % круговая стрелка τ_i
        tau_i = tau_req(i);
        rad_arc = abs(tau_i) * arrow_sc;
        if rad_arc > 0.015 * arm_reach
            draw_arc_arrow(ax3, org, z, rad_arc, sign(tau_i), cmap(i,:));
        end

        % метка
        off = org + 0.055*arm_reach * (z/norm(z) + [0.4;0.25;0.15]);
        text(ax3, off(1), off(2), off(3), ...
             sprintf(' J%d\n%.1f Нм', i, tau_i), ...
             'Color', cmap(i,:), 'FontSize', 8, 'FontWeight','bold');
    end

    % -- Вектор g ------------------------------------------------------------
    g_orig = joints(:,1) + [-0.03; 0; 0] * arm_reach;
    g_len  = 0.14 * arm_reach;
    draw_arrow3d(ax3, g_orig, g_orig + g_len*[0;0;-1], ...
                 0.005*arm_reach, [1 0.87 0.34], 1.0);
    text(ax3, g_orig(1), g_orig(2), g_orig(3) - g_len - 0.015*arm_reach, ...
         ' g', 'Color','#ffdd57','FontSize',11,'FontWeight','bold');

    % -- TCP frame (малые XYZ стрелки) ---------------------------------------
    draw_frame(ax3, T{n+1}, 0.06*arm_reach, 0.003*arm_reach);

    % ═════════════════════════════════════════════════════════════════════
    % ── ПРАВАЯ ПАНЕЛЬ: Bar-chart ─────────────────────────────────────────
    ax2 = subplot('Position',[0.61 0.10 0.36 0.82], 'Parent', fig);
    hold(ax2,'on'); grid(ax2,'on'); box(ax2,'on');
    set(ax2, 'Color','#16213e','GridColor','#0f3460','GridAlpha',0.45, ...
             'XColor','w','YColor','w','FontSize',9, ...
             'XLim',[0.4, n+0.6], 'YDir','normal');

    % Границы приводов (серые пунктиры)
    tau_max_abs = robot.joint_tau_max;
    for i = 1:n
        plot(ax2, [i-0.35 i+0.35], [ tau_max_abs(i)  tau_max_abs(i)], ...
             '--', 'Color',[0.7 0.7 0.7 0.6], 'LineWidth',1);
        plot(ax2, [i-0.35 i+0.35], [-tau_max_abs(i) -tau_max_abs(i)], ...
             '--', 'Color',[0.7 0.7 0.7 0.6], 'LineWidth',1);
    end

    % τ_static (прозрачный)
    for i = 1:n
        bar(ax2, i, tau_static(i), 0.55, ...
            'FaceColor', cmap(i,:), 'EdgeColor','none', 'FaceAlpha', 0.30);
    end

    % τ_req (непрозрачный) — LineWidth передаётся только при наличии обводки
    for i = 1:n
        util_i = abs(tau_req(i)) / tau_max_abs(i);
        if util_i >= 1.0
            bar(ax2, i, tau_req(i), 0.40, ...
                'FaceColor', cmap(i,:), 'EdgeColor', [1 0.2 0.2], ...
                'LineWidth', 2.0, 'FaceAlpha', 0.90);
        elseif util_i >= 0.80
            bar(ax2, i, tau_req(i), 0.40, ...
                'FaceColor', cmap(i,:), 'EdgeColor', [1 0.85 0.1], ...
                'LineWidth', 1.5, 'FaceAlpha', 0.90);
        else
            bar(ax2, i, tau_req(i), 0.40, ...
                'FaceColor', cmap(i,:), 'EdgeColor', 'none', ...
                'FaceAlpha', 0.90);
        end
    end

    % Нулевая линия
    yline(ax2, 0, 'Color','w','LineWidth',0.8,'LineStyle','-');

    % Значения над столбцами
    y_off = 0.025 * max(abs([tau_req(:); tau_static(:)]));
    for i = 1:n
        util_i = abs(tau_req(i)) / tau_max_abs(i);
        v = tau_req(i);
        col_txt = 'w';
        if util_i >= 1.0;   col_txt = '#ff4444'; end
        if util_i >= 0.80 && util_i < 1.0; col_txt = '#ffd700'; end
        text(ax2, i, v + sign(v)*y_off, ...
             sprintf('%.1f\n(%.0f%%)', v, util_i*100), ...
             'HorizontalAlignment','center','Color',col_txt, ...
             'FontSize',8,'FontWeight','bold');
    end

    % Пороги 80%
    for i = 1:n
        th = 0.80 * tau_max_abs(i);
        plot(ax2, [i-0.28 i+0.28], [th th], ':', 'Color',[1 0.85 0.1 0.5], ...
             'LineWidth',1.2);
        plot(ax2, [i-0.28 i+0.28], [-th -th], ':', 'Color',[1 0.85 0.1 0.5], ...
             'LineWidth',1.2);
    end

    xlabel(ax2, 'Сустав', 'Color','w','FontSize',11);
    ylabel(ax2, 'Момент [Н·м]', 'Color','w','FontSize',11);
    title(ax2, sprintf('Статические моменты  (k_{dyn} = %.1f)', robot.k_dyn), ...
          'Color','w','FontSize',12,'FontWeight','bold');
    xticks(ax2, 1:n);
    xlbls = cell(1,n);
    for i=1:n; xlbls{i} = sprintf('J%d', i); end
    xticklabels(ax2, xlbls);

    % Легенда
    p1 = bar(ax2, NaN, NaN, 'FaceColor',[0.6 0.6 0.6],'FaceAlpha',0.30,'EdgeColor','none');
    p2 = bar(ax2, NaN, NaN, 'FaceColor',[0.6 0.6 0.6],'FaceAlpha',0.90,'EdgeColor','none');
    legend(ax2, [p1 p2], {'τ_{static}','τ_{req} = k_{dyn}·τ_{static}'}, ...
           'TextColor','w','Color','#0f3460','EdgeColor','none', ...
           'FontSize',9,'Location','best');

    sgtitle(fig, 'Анализ статических моментов роборуки', ...
            'Color','w','FontSize',15,'FontWeight','bold', ...
            'BackgroundColor','#1a1a2e');
end


% ═══════════════════════════════════════════════════════════════════════════
%  ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ
% ═══════════════════════════════════════════════════════════════════════════

function draw_base_plate(ax, origin, arm_reach)
    sz  = 0.12 * arm_reach;
    N   = 20;
    [bx,by] = meshgrid(linspace(-sz,sz,N), linspace(-sz,sz,N));
    bz = origin(3) * ones(N,N);
    bx = bx + origin(1);
    by = by + origin(2);
    surf(ax, bx, by, bz, 'FaceColor','#2c3e50','EdgeColor','none','FaceAlpha',0.7);
end

function draw_tube(ax, p1, p2, r, color)
    v   = p2 - p1;
    len = norm(v);
    if len < 1e-9; return; end
    v = v / len;

    [cx, cy, cz] = cylinder(r, 18);
    cz = cz * len;

    R   = rot_align_z(v);
    pts = R * [cx(:)'; cy(:)'; cz(:)'] + p1;
    surf(ax, reshape(pts(1,:),size(cx)), reshape(pts(2,:),size(cy)), ...
             reshape(pts(3,:),size(cz)), ...
         'FaceColor',color,'EdgeColor','none','FaceAlpha',0.92,...
         'AmbientStrength',0.5,'DiffuseStrength',0.8);
end

function draw_sphere(ax, center, r, color)
    [sx,sy,sz] = sphere(14);
    surf(ax, center(1)+r*sx, center(2)+r*sy, center(3)+r*sz, ...
         'FaceColor',color,'EdgeColor','none','FaceAlpha',0.88, ...
         'AmbientStrength',0.5,'DiffuseStrength',0.7);
end

function draw_arrow3d(ax, p0, p1, r, color, alpha)
    v   = p1 - p0;
    len = norm(v);
    if len < 1e-9; return; end
    frac_shaft = 0.78;
    p_mid = p0 + frac_shaft * v;
    draw_tube_alpha(ax, p0,   p_mid, r,          color, alpha);
    draw_cone(ax,       p_mid, p1,   2.8*r,       color, alpha);
end

function draw_tube_alpha(ax, p1, p2, r, color, alpha)
    v   = p2 - p1;
    len = norm(v);
    if len < 1e-9; return; end
    v = v / len;
    [cx,cy,cz] = cylinder(r, 12);
    cz = cz * len;
    R  = rot_align_z(v);
    pts = R * [cx(:)'; cy(:)'; cz(:)'] + p1;
    surf(ax, reshape(pts(1,:),size(cx)), reshape(pts(2,:),size(cy)), ...
             reshape(pts(3,:),size(cz)), ...
         'FaceColor',color,'EdgeColor','none','FaceAlpha',alpha);
end

function draw_cone(ax, p_base, p_tip, r, color, alpha)
    v   = p_tip - p_base;
    len = norm(v);
    if len < 1e-9; return; end
    v   = v / len;
    th  = linspace(0, 2*pi, 20);
    cx  = [r*cos(th); zeros(1,20)];
    cy  = [r*sin(th); zeros(1,20)];
    cz  = [zeros(1,20); len*ones(1,20)];
    R   = rot_align_z(v);
    pts = R * [cx(:)'; cy(:)'; cz(:)'] + p_base;
    surf(ax, reshape(pts(1,:),size(cx)), reshape(pts(2,:),size(cy)), ...
             reshape(pts(3,:),size(cz)), ...
         'FaceColor',color,'EdgeColor','none','FaceAlpha',alpha);
end

function draw_arc_arrow(ax, origin, z_axis, radius, direction, color)
    z = z_axis / norm(z_axis);

    % Перпендикуляр к z_axis
    if abs(z(3)) < 0.9
        perp = cross(z, [0;0;1]);
    else
        perp = cross(z, [1;0;0]);
    end
    perp  = perp / norm(perp);
    perp2 = cross(z, perp);

    ang = linspace(0, direction * 1.55 * pi, 80);
    arc = radius * (cos(ang) .* perp + sin(ang) .* perp2) + origin;

    % Тело дуги
    plot3(ax, arc(1,:), arc(2,:), arc(3,:), ...
          'Color', [color, 0.85], 'LineWidth', 2.8);

    % Конус-наконечник
    tip  = arc(:,end);
    tang = arc(:,end) - arc(:,end-2);
    if norm(tang) < 1e-9; return; end
    tang = tang / norm(tang);
    cone_len = max(radius * 0.28, 0.005);
    draw_cone(ax, tip, tip + cone_len*tang, 0.10*radius, color, 1.0);
end

function draw_frame(ax, T4x4, len, r)
    orig  = T4x4(1:3,4);
    colors = {'#e74c3c','#2ecc71','#3498db'};   % X=red Y=green Z=blue
    for k = 1:3
        dir = T4x4(1:3,k);
        draw_arrow3d(ax, orig, orig + len*dir, r, colors{k}, 1.0);
    end
end

function R = rot_align_z(v)
% Матрица вращения, которая поворачивает ось Z на вектор v.
    z0 = [0;0;1];
    v  = v / norm(v);
    ax_r = cross(z0, v);
    if norm(ax_r) < 1e-9
        R = eye(3) * sign(dot(z0,v));
        return;
    end
    ax_r = ax_r / norm(ax_r);
    ang  = acos(max(-1, min(1, dot(z0,v))));
    R    = rot_from_axis_angle(ax_r, ang);
end

function R = rot_from_axis_angle(ax, ang)
    ax = ax / norm(ax);
    K  = [ 0,     -ax(3),  ax(2);
           ax(3),  0,     -ax(1);
          -ax(2),  ax(1),  0    ];
    R  = eye(3) + sin(ang)*K + (1-cos(ang))*(K*K);
end
