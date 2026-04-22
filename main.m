% =========================================================================
%   MAIN.M — Статический анализ моментов 6-DOF роборуки
%
%   Запуск:  >> main
%
%   Структура проекта:
%     robot_def.m          — параметры робота
%     forward_kinematics.m — прямая кинематика (DH)
%     static_torque.m      — расчёт τ_static и τ_req
%     visualize.m          — 3D-визуализация + bar-chart
%     workspace_analysis.m — тепловые карты по рабочему пространству
% =========================================================================

clc; clear; close all;

fprintf('=================================================================\n');
fprintf('   Статический анализ моментов 6-DOF роборуки\n');
fprintf('=================================================================\n\n');

% ── 1. Инициализация робота ───────────────────────────────────────────────
robot = robot_def();

fprintf('  Робот: %d DOF\n', robot.n);
fprintf('  Масса звеньев:   [%s] кг\n', num2str(robot.mass,'%.2f  '));
fprintf('  Нагрузка:         %.1f кг\n', robot.payload_mass);
fprintf('  Коэф. k_dyn:      %.2f\n\n', robot.k_dyn);

% ── 2. Анализ нескольких поз ─────────────────────────────────────────────
%   Описание поз: [q1, q2, q3, q4, q5, q6] в градусах
poses_deg = {
    'Вертикаль (home)',        [  0,    0,    0,    0,    0,    0];
    'Вперёд горизонтально',    [  0,  -90,    0,  -90,    0,    0];
    'Рабочая поза',            [  0,  -45,   60,    0,   45,    0];
    'Крайнее боковое',         [ 90,  -60,   45,  -30,   30,    0];
    'Нижнее положение',        [  0,  -90,   90,    0,    0,    0];
};

n_poses = size(poses_deg, 1);
all_tau_req = zeros(robot.n, n_poses);

fprintf('  %-28s', 'Поза');
for i = 1:robot.n
    fprintf('  J%d [Нм]', i);
end
fprintf('  max_util\n');
fprintf('  %s\n', repmat('-', 1, 28 + robot.n*11 + 12));

for p = 1:n_poses
    name_p = poses_deg{p,1};
    q_deg  = poses_deg{p,2};
    q_rad  = deg2rad(q_deg);

    [tau_s, tau_r, util] = static_torque(robot, q_rad);
    all_tau_req(:, p) = tau_r;

    fprintf('  %-28s', name_p);
    for i = 1:robot.n
        fprintf('  %7.1f', tau_r(i));
    end
    fprintf('  %5.0f%%\n', max(util)*100);
end

% ── 3. Подробный вывод выбранной позы ────────────────────────────────────
fprintf('\n--- Детальный анализ: "Рабочая поза" ---\n\n');
q_main = deg2rad(poses_deg{3,2});
[tau_static, tau_req, util, p_com_w] = static_torque(robot, q_main);

fprintf('  %-6s  %-22s  %-14s  %-14s  %-14s  %s\n', ...
        'Сустав', 'Имя', 'τ_static [Нм]', 'τ_req [Нм]', ...
        'τ_max [Нм]', 'Загрузка');
fprintf('  %s\n', repmat('-', 1, 92));
for i = 1:robot.n
    bar_len = round(util(i) * 20);
    bar_str = [repmat('█', 1, min(bar_len,20)), repmat('░', 1, max(0,20-bar_len))];
    if util(i) >= 1.0;      warn = ' ⚠ ПЕРЕГРУЗ';
    elseif util(i) >= 0.80; warn = ' △ внимание';
    else;                   warn = '';
    end
    fprintf('  J%-5d %-22s  %12.2f    %12.2f    %12.0f    %s %4.0f%%%s\n', ...
            i, robot.joint_names{i}, tau_static(i), tau_req(i), ...
            robot.joint_tau_max(i), bar_str, util(i)*100, warn);
end

fprintf('\n  Позиции CoM звеньев в мировой СК [м]:\n');
fprintf('  %-8s  %8s  %8s  %8s\n', 'Звено', 'X', 'Y', 'Z');
for i = 1:robot.n
    fprintf('  %-8d  %8.4f  %8.4f  %8.4f\n', i, p_com_w(:,i)');
end
fprintf('  %-8s  %8.4f  %8.4f  %8.4f  (нагрузка)\n', ...
        'Payload', p_com_w(:,end)');

% ── 4. Визуализация ──────────────────────────────────────────────────────
fprintf('\n  Отрисовка 3D-визуализации...\n');
visualize(robot, q_main, tau_req, tau_static);

% ── 5. Анализ рабочего пространства (J2 vs J3) ───────────────────────────
fprintf('  Анализ рабочего пространства (J2 × J3)...\n');
workspace_analysis(robot, 2, 3, 40);

% ── 6. Критическая конфигурация — поиск максимума τ_req(J2) ─────────────
fprintf('\n  Поиск наихудшей конфигурации для J2...\n');

objective = @(q) -abs(static_torque(robot, q));
% Оптимизируем J1..J6 в диапазоне ±π/2
lb = -ones(1,robot.n)*pi/2;
ub =  ones(1,robot.n)*pi/2;
opts = optimset('Display','off','TolX',1e-4,'TolFun',1e-4);

q0 = zeros(1,robot.n);
q_worst = fminsearch(@(q) -max(abs(static_torque_vec(robot,q))), q0, opts);

[~, tau_worst, util_worst] = static_torque(robot, q_worst);
fprintf('  Наихудшая поза: [%s] °\n', num2str(rad2deg(q_worst),'%.1f '));
fprintf('  max τ_req = %.1f Н·м   max utilization = %.0f%%\n\n', ...
        max(abs(tau_worst)), max(util_worst)*100);

fprintf('=================================================================\n');
fprintf('  Готово. Откройте графики для анализа.\n');
fprintf('=================================================================\n');


% ── Вспомогательная функция для fminsearch ────────────────────────────────
function tau_vec = static_torque_vec(robot, q)
    [~, tau_req] = static_torque(robot, q);
    tau_vec = tau_req;
end
