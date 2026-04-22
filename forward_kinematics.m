function T = forward_kinematics(DH, q)
% FORWARD_KINEMATICS  Вычисляет цепочку однородных трансформаций T{1..n+1}.
%
%   T = forward_kinematics(DH, q)
%
%   DH  — матрица DH-параметров [n x 4]: [a, alpha, d, theta_off]
%   q   — вектор углов суставов [1 x n] или [n x 1], рад
%
%   Возвращает cell-массив T{1..n+1}:
%     T{1}    = eye(4)               — мировая СК (база)
%     T{i+1}  = T{i} * A_i(q_i)     — СК звена i относительно базы
%
%   Соглашение: модифицированная DH (стандарт Craig / UR):
%
%     A_i = Rot_x(alpha_{i-1}) * Trans_x(a_{i-1}) *
%           Trans_z(d_i) * Rot_z(theta_i)
%
%   В данной реализации используется стандартная (Denavit-Hartenberg):
%
%     A_i = Rot_z(theta_i) * Trans_z(d_i) *
%           Trans_x(a_i) * Rot_x(alpha_i)

    q = q(:);                           % гарантируем column-vector
    n = size(DH, 1);

    T      = cell(n+1, 1);
    T{1}   = eye(4);

    for i = 1:n
        a     = DH(i, 1);
        alpha = DH(i, 2);
        d     = DH(i, 3);
        theta = DH(i, 4) + q(i);       % theta_offset + joint variable

        ct = cos(theta);  st = sin(theta);
        ca = cos(alpha);  sa = sin(alpha);

        % Стандартная DH матрица A_i
        Ai = [ ct,  -st*ca,   st*sa,   a*ct;
               st,   ct*ca,  -ct*sa,   a*st;
                0,      sa,      ca,      d;
                0,       0,       0,      1 ];

        T{i+1} = T{i} * Ai;
    end
end
