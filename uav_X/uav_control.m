function [controls, reg_state] = uav_control(uav_state, flight_mission, reg_state)
    %% Входные параметры
    % Координаты в инерциальной СК
    x = uav_state.coords(1); y = uav_state.coords(2); z = uav_state.coords(3);
    % Углы ориентации
    roll = uav_state.roll; pitch = uav_state.pitch; yaw = uav_state.yaw;
    % Линейные скорости в инерциальной СК
    Vx = uav_state.V(1); Vy = uav_state.V(2); Vz = uav_state.V(3);
    % Угловые скорости в инерциальной СК
    Wx = uav_state.W(1); Wy = uav_state.W(2); Wz = uav_state.W(3);
    % Угловые скорости
    droll = uav_state.droll; dpitch = uav_state.dpitch; dyaw = uav_state.dyaw;
    
    W0 = reg_state.W0;
    dt = reg_state.dt;
    
    % Матрица перехода от связанной СК к её проекции на плоскость горизонта
    % (матрица поворота без угла курса)
    Rbgt = [cosd(pitch), -sind(pitch)*cosd(roll), sind(pitch)*sind(roll);
        sind(pitch), cosd(pitch)*cosd(roll), -cosd(pitch)*sind(roll);
        0, sind(roll), cosd(roll)];
    
    % Проекции скоростей движения БЛА на плоскость горизонта
    Vgt = Rbgt * uav_state.Vb;
    % Продольная скорость движения
    V_forward = Vgt(1);
    % Боковая скорость движения
    V_right = Vgt(3);
    
    %% Конструктивные параметры
    k = 1.4851*10^-4; % k - подъемный коэффициен
    l = 0.36/2;
    b = 0.0002; % коэффициент реактивного момента
    
    % Ограничения на скорость вращения винтов
    Wmin = 50; % ~480 об/мин
    Wmax = 200; % ~1900 об/мин
    dWmax = Wmax - W0;
    dWmin = Wmin - W0;
    
    %% Управление по продольной скорости
    vf_error = flight_mission.V(1) - V_forward;
    pitch_need = -(1.2 * vf_error + 0.03 * reg_state.vf.D);
    reg_state.vf.D = vf_error - reg_state.vf.D;
    reg_state.vf.I = vf_error * dt + reg_state.vf.I;
    
    %% Управление по боковой скорости
    vr_error = flight_mission.V(3) - V_right;
    roll_need = 1.2 * vr_error + 0.03 * reg_state.vr.D;
    reg_state.vr.D = vr_error - reg_state.vr.D;
    reg_state.vr.I = vr_error * dt + reg_state.vr.I;
    
    %% Управление по вертикальной скорости
    y_error = flight_mission.coords(2) - y;
    Vy_need = 0.8 * y_error;
    Vy_error = Vy_need - Vy;
    delta_Fy = 0.1 * Vy_error + 0.006 * reg_state.vy.I;
    reg_state.vy.D = Vy_error - reg_state.vy.D;
    reg_state.vy.I = Vy_error * dt + reg_state.vy.I;
    
    %% Управление по углу тангажа
%     pitch_error = flight_mission.pitch - pitch;
    pitch_error = pitch_need - pitch;
    dpitch_need = 0.1 * pitch_error + 0.0 * reg_state.pitch.D + 0.001 * reg_state.pitch.I;
    dpitch_error = dpitch_need - dpitch;
    delta_Mz = 4e-5 * dpitch_error;
    reg_state.pitch.D = pitch_error - reg_state.pitch.D;
    reg_state.pitch.I = pitch_error * dt + reg_state.pitch.I;
    
    %% Управление по углу крена
%     roll_error = flight_mission.roll - roll;
    roll_error = roll_need - roll;
    droll_need = 0.1 * roll_error + 0.0 * reg_state.roll.D + 0.001 * reg_state.roll.I;
    droll_error = droll_need - droll;
    delta_Mx = 4e-5 * droll_error;
    reg_state.roll.D = roll_error - reg_state.roll.D;
    reg_state.roll.I = roll_error * dt + reg_state.roll.I;
    
    %% Управление по углу курса
    yaw_error = flight_mission.yaw - yaw;
    dyaw_need = 0.04 * yaw_error + 0.01 * reg_state.yaw.D + 0.001 * reg_state.yaw.I;
    dyaw_need = min(max(dyaw_need, -0.5), 0.5);
    dyaw_error = dyaw_need - dyaw;
    delta_My = 2e-4 * dyaw_error + 0 * reg_state.dyaw.D + 0 * reg_state.dyaw.I;
    reg_state.dyaw.D = dyaw_error - reg_state.dyaw.D;
    reg_state.dyaw.I = dyaw_error * dt + reg_state.dyaw.I;
    reg_state.yaw.D = yaw_error - reg_state.yaw.D;
    reg_state.yaw.I = yaw_error * dt + reg_state.yaw.I;
    
    %% Решение системы уравнений и формирования угловых скоростей вращения 
    % винтов в зависимости от необходимых величин изменения моментов и
    % тяги
    dW1 = (b*delta_Mx + b*delta_Mz + l*b*delta_Fy + l*delta_My*k)/(4*l*b*k);
    dW2 = (b*delta_Mx - b*delta_Mz + l*b*delta_Fy - l*delta_My*k)/(4*l*b*k);
    dW3 = -(b*delta_Mx + b*delta_Mz - l*b*delta_Fy - l*delta_My*k)/(4*l*b*k);
    dW4 = -(b*delta_Mx - b*delta_Mz - l*b*delta_Fy + l*delta_My*k)/(4*l*b*k);
    % Отрицательное значение dW1 (квадарата скорости) означает, что скорость
    % этого мотора надо снизить. Поэтому выносим минус за знак квадратного
    % корня
    if dW1 >= 0,  dW1 = sqrt(dW1); else, dW1 = -sqrt(abs(dW1)); end
    if dW2 >= 0,  dW2 = sqrt(dW2); else, dW2 = -sqrt(abs(dW2)); end
    if dW3 >= 0,  dW3 = sqrt(dW3); else, dW3 = -sqrt(abs(dW3)); end
    if dW4 >= 0,  dW4 = sqrt(dW4); else, dW4 = -sqrt(abs(dW4)); end
    % Ограничения на максимальную и минимальную скорость вращения моторов
    % предотвращает вращение моторов в обратную сторону
    dW1 = min(max(dW1, dWmin), dWmax);
    dW2 = min(max(dW2, dWmin), dWmax);
    dW3 = min(max(dW3, dWmin), dWmax);
    dW4 = min(max(dW4, dWmin), dWmax);
    % Добавляем полученное изменение скоростей вращения к W0
    % Округляем скорость вращения винтов до 0.1 рад/с (практическое
    % ограничение)
    W1 = round(W0 + dW1, 1);
    W2 = round(W0 + dW2, 1);
    W3 = round(W0 + dW3, 1);
    W4 = round(W0 + dW4, 1);
    
    controls = [W1, W2, W3, W4];
end

