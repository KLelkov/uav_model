function uav_state = uav_model_X(uav_state, controls, dt)
    %% Входные параметры
    % Координаты в инерциальной СК
    x = uav_state.coords(1); y = uav_state.coords(2); z = uav_state.coords(3);
    % Углы ориентации
    roll = uav_state.roll; pitch = uav_state.pitch; yaw = uav_state.yaw;
    % Линейные скорости в связанной СК
    Vbx = uav_state.Vb(1); Vby = uav_state.Vb(2); Vbz = uav_state.Vb(3);
    % Угловые скорости в связанной СК
    Wbx = uav_state.Wb(1); Wby = uav_state.Wb(2); Wbz = uav_state.Wb(3);
    W1 = controls(1); W2 = controls(2);
    W3 = controls(3); W4 = controls(4);
    
    %% Конструктивные параметры
    a = 0.36; % длина квадрокоптера и ширина
    k = 1.4851*10^-4; % k - подъемный коэффициен
    h = 0.07; % высота квадрокоптра
    L1 = 485/2/1000; % расстояние от винта квадрокоптера до цента
    % Когда винты располагаются схема Х, плечо моторов не равно длине плеча L
    l = 0.36/2;
    m = 0.7; % масса квадрокоптера
    g = 9.81; % ускорение свободного падения
    Ix = 1/12*m*(3*L1^3+h^2); % момент инерции
    Iz = Ix; % момент инерции
    Iy = m*L1^2/2; % момент инерции 
    Irot = 0; % момент инерции ротора
    Sx = a*h; Sz = a*h; Sy = a*a; % площади поверхностей квадрокоптера
    cd = 0.1; % аэродинамический коэффициент
    md = 0.00004; % аэродинамический коэффициент
    ro = 1.2; % плотность воздуха
    n = 105/2;
    b = 0.0002; % коэффициент реактивного момента
    
    %% Линейное движение
    % Проекции силы тяжести на оси связанной СК
    Fgb = [-m*g*sind(pitch);
           -m*g*cosd(pitch)*cosd(roll);
           m*g*cosd(pitch)*sind(roll)];
    % Проекции тяги моторов на оси связанной СК
    Fpb = [0;
           k*(W1^2+W2^2+W3^2+W4^2);
           0];
    % Проекции сил аэродинамического сопротивления на оси связанной СК
    Fab = [ -sign(Vbx)*cd*ro*Vbx^2*Sx/2;
            -sign(Vby)*cd*ro*Vby^2*Sy/2;
            -sign(Vbz)*cd*ro*Vbz^2*Sz/2];
    % Ускорение Кориолеса в связанной СК
    Acor = 2*[Wby*Vbz - Wbz*Vby;
              Wbz*Vbx - Wbx*Vbz;
              Wbx*Vby - Wby*Vbx];
    % Уравнение линейного движения БЛА
    dVb = (Fgb + Fpb + Fab)/m - Acor;
    
    %% Вращательное движение
    % Моменты от тяги моторов в связанной СК
    Mpb = [k*l*(W1^2 + W2^2 - W3^2 - W4^2);
           b*(W1^2 + W3^2 - W2^2 - W4^2);
           k*l*(W1^2 + W4^2 - W2^2 - W3^2)];
    % Моменты аэродинамических сил в связанной СК
    Mab = [ sign(Vbx)*md*ro*Vbx^2*Sx*n/2;
            sign(Vby)*md*ro*Vby^2*Sy*n/2;
            sign(Vbz)*md*ro*Vbz^2*Sz*n/2];
    % Гироскопические моменты, вызванные собственным вращением БЛА
    Mgyrb = [Wby*Wbz*(Iy - Iz);
             Wbx*Wbz*(Ix - Iz);
             Wbx*Wby*(Iy - Ix);];
    % Гироскопические моменты вызванные вращением моторов БЛА
    Mgpb = [Irot*Wbz*(W2 + W4 - W1 - W3);
            0;
            Irot*Wbx*(W1 + W3 - W2 - W4);];
    % Уравнение углового движения БЛА
    dWb = (Mpb + Mab + Mgyrb + Mgpb) ./ [Ix; Iy; Iz];
    
    %% Расчёт линейных и угловых скоростей в связанной СК путём интегирования
    Vbx = Vbx + dVb(1)*dt;
    Vby = Vby + dVb(2)*dt;
    Vbz = Vbz + dVb(3)*dt;
    
    Wbx = Wbx + dWb(1)*dt;
    Wby = Wby + dWb(2)*dt;
    Wbz = Wbz + dWb(3)*dt;
    
    %% Расчёт навигационных параметров в инерциальной СК
    % Матрица поворота от связанной СК к инерциальной СК
    Rbg = [cosd(yaw)*cosd(pitch), -cosd(yaw)*sind(pitch)*cosd(roll)+sind(yaw)*sind(roll), cosd(yaw)*sind(pitch)*sind(roll)+sind(yaw)*cosd(roll);
        sind(pitch), cosd(pitch)*cosd(roll), -cosd(pitch)*sind(roll);
        -sind(yaw)*cosd(pitch), cosd(yaw)*sind(roll)+sind(yaw)*sind(pitch)*cosd(roll), cosd(yaw)*cosd(roll)-sind(yaw)*sind(pitch)*sind(roll)];
    % Линейные скорости в инерциальной СК
    V = Rbg * [Vbx; Vby; Vbz];
    % Линейные скорости в инерциальной СК
    W = Rbg * [Wbx; Wby; Wbz];
    % Угловые скорости в инерциальной СК
    droll = Wbx - (Wby*cosd(roll) - Wbz*sind(roll)) * tand(pitch);
    dpitch = Wbz*cosd(roll) + Wby*sind(roll);
    dyaw = (Wby*cosd(roll) - Wbz*sind(roll)) / cosd(pitch);
    
    % Навигационные параметры
    x = x + V(1)*dt;
    y = y + V(2)*dt;
    z = z + V(3)*dt;
    roll = roll + rad2deg(droll*dt);
    pitch = pitch + rad2deg(dpitch*dt);
    yaw = yaw + rad2deg(dyaw*dt);
    
    %% Формирование выходной структуры
    % Координаты в инерциальной СК
    uav_state.coords = [x; y; z];
    % Углы ориентации
    uav_state.roll = roll;
    uav_state.pitch = pitch;
    uav_state.yaw = yaw; 
    % Линейные скорости в связанной СК
    uav_state.Vb = [Vbx; Vby; Vbz];
    % Угловые скорости в связанной СК
    uav_state.Wb = [Wbx; Wby; Wbz];
    % Линейные скорости в инерциальной СК
    uav_state.V = V;
    % Угловые скорости в инерциальной СК
    uav_state.W = W;
    uav_state.droll = droll;
    uav_state.dpitch = dpitch;
    uav_state.dyaw = dyaw;
    
end

