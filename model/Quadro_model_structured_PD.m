clear;
clc
%% Параметры моделирования
start_time = 0;
end_time = 30;
dt = 0.005; % шаг дискретизации
nSim = (end_time - start_time)/ dt; % количество точек при моделировании
times = start_time:dt:end_time;
% Массивы для сбора логов и построения графиков
Coords = zeros(nSim, 3);
Speed = zeros(nSim, 3);
Acceleration = zeros(nSim, 3);
Angles = zeros(nSim, 3);
Rates = zeros(nSim, 3);
Time = zeros(nSim, 1);
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
%% Моделирование шумов
% Simulate some disturbance in the angular velocity.
% The magnitude of the deviation is in radians / second.
% deviation = 100;
% thetadot = deg2rad(2 * deviation * rand(3, 1) - deviation);

 %% Начальные условия
x = [0; 0; 10]; % X Y Z
xdot = [0; 0; 0]; % Vx Vy Vz
theta = [5; 0; 0]; % Крен Тангаж Курс
thetadot = [0;0;0];
theta_need =[0;0;0];% желаемые крен, тангаж,курс
thetadot_need=[0;0;0];%желаемые угловые скорости
dW = 50; % изменение угловой скорости для придания ускорения БЛА
% Скорости вращения винтов
%i = [W_hover+dW, W_hover+dW, W_hover+dW, W_hover+dW];
%% Основной цикл
for j = 1:nSim
    % Расчёт угловых скоростей в связанной (body) СК
    i = PD_controller(theta_need,theta,thetadot_need,thetadot,m,g,k,b,I,L);
    omega = thetadot2omega(thetadot, theta);
    % Расчёт линейных ускорений
    a = acceleration(i, theta, xdot, m, g, k, cd,S,r0);
    % Расчёт угловых ускорений
    omegadot = angular_acceleration(i, omega, I, L, b, k);
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
end
%% Построение графиков
close all
figure('Name', 'Координаты')
plot(Time, Coords, 'LineWidth', 2);
grid on
xlabel('Время, с')
ylabel('Координата, м')
legend X Y Z
title Координаты

figure('Name', 'Скорости и ускорения')
subplot(2,1,1)
plot(Time, Speed, 'LineWidth', 2);
grid on
xlabel('Время, с')
ylabel('Скорость, м/с')
legend Vx Vy Vz
title 'Скорости'
subplot(2,1,2)
plot(Time, Acceleration, 'LineWidth', 2);
grid on
xlabel('Время, с')
ylabel('Ускорение, м/с2')
legend ax ay az
title 'Ускорения'
figure('Name', 'Углы и угловые скорости')
subplot(2,1,1)
plot(Time, Angles, 'LineWidth', 2);
grid on
xlabel('Время, с')
ylabel('Угол, град')
legend Крен Тангаж Курс
title 'Углы ориентации'
subplot(2,1,2)
plot(Time, Rates, 'LineWidth', 2);
grid on
xlabel('Время, с')
ylabel('Угловая скорость, ???')
legend wx wy wz
title 'Угловые скорости'


