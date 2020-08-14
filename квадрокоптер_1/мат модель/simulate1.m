function [cost,x,xdot,theta,thetadot] = simulate1(params,j,theta,thetadot,Dthetadot,theta_need,thetadot_need,x,xdot,dt,end_time,start_time)
  %% Конструктивные параметры
    m = 0.3;
    a = 0.245;%длина квадрокоптера
    width = 0.290;%ширина квадрокоптера
    h = 0.055; %высота квадрокоптера
    S = zeros(3,3);%площадь квадрокоптера
    S(1) = a*h;%Sx
    S(2) = width*h;%Sy
    S(3)= width*a;%Sz
    % L - плечо от центра квадрокоптера до винтов
    L = sqrt((a/2)^2+(width/2)^2);
    Ix = 1/12*m*(3*L^3+h^2);
    Iy = Ix;
    Iz = m*L^2/2;
    % I - моменты инерции, придумал сам
    I =[Ix 0 0;0 Iy 0; 0 0 Iz];
    %% Постоянные величины
    g = 9.81;
    r0 =1.2;%плотность воздуха
    %сd - коэффициент аэродинамической силы
    cd =0.4;
    %b - коэффициент крутящего момента роторов. k - подъемный коэффициент
    b =0.7426*10^-6;
    k = 1.4851*10^-5;
    % Необходимая уголовая скорость вращения винтов для зависания на месте
    W_hover = 49257.0198; % квадрат угловой скорости
 % Расчёт угловых скоростей в связанной (body) СК
        omega = thetadot2omega(thetadot, theta);
        i = controller(theta_need,theta,thetadot_need,thetadot,m,g,k,b,I,L,Dthetadot, params);
        % Расчёт линейных ускорений
        a = acceleration(i, theta, xdot, m, g, k, cd,S,r0);
        % Расчёт угловых ускорений
        omegadot = angular_acceleration(i, omega, I, L, b, k);
        Dthetadot = omega2thetadot(omegadot, theta);
        % Интегрирование угловых ускорений
        omega = omega + dt * omegadot;
        % Расчёт угловых скоростей в инерицальной (inertial) СК
        thetadot = omega2thetadot(omega, theta);
        % Расчёт углов ориентации
        theta = theta + dt * thetadot;
        % Расчёт скоростей движения
        xdot = xdot + dt * a;
        % Расчёт координат
        x = x + dt * xdot;
        % Сбор логов для графиков
        Coords(j,:) = x(:,1);
        Speed(j,:) = xdot(:,1);
        Acceleration(j,:) = a(:,1);
        Angles(j,:) = theta;
        Rates(j,:) = thetadot;
        Time(j) = j*dt; % Расчёт угловых скоростей в связанной (body) СК
        omega = thetadot2omega(thetadot, theta);
        i = controller(theta_need,theta,thetadot_need,thetadot,m,g,k,b,I,L,Dthetadot, params);
        % Расчёт линейных ускорений
        a = acceleration(i, theta, xdot, m, g, k, cd,S,r0);
        % Расчёт угловых ускорений
        omegadot = angular_acceleration(i, omega, I, L, b, k);
        Dthetadot = omega2thetadot(omegadot, theta);
        % Интегрирование угловых ускорений
        omega = omega + dt * omegadot;
        % Расчёт угловых скоростей в инерицальной (inertial) СК
        thetadot = omega2thetadot(omega, theta);
        % Расчёт углов ориентации
        theta = theta + dt * thetadot;
        % Расчёт скоростей движения
        xdot = xdot + dt * a;
        % Расчёт координат
        x = x + dt * xdot;
        % Сбор логов для графиков
        Coords(j,:) = x(:,1);
        Speed(j,:) = xdot(:,1);
        Acceleration(j,:) = a(:,1);
        Angles(j,:) = theta;
        Rates(j,:) = thetadot;
        Time(j) = j*dt;     
    % Функция стоимости (ошибки) при использовании заданых коэффициентов
    % управления
    cost = 1/(end_time - start_time) * sum((theta_need - theta).^2) * dt;
    
end