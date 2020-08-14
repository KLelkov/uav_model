clear;
clc;
% Параметры подстройки
delta = 0.8;
alfa = 1.2;
% Вектор инаправления поиска коэффициентов
var = [0.7, 0.3, 0]; %Uk
% Случайное распределение начальных коэффициентов
params0 = [rand()*3, rand()*3, rand()*2];
% params0 = [2, 1.4, 0.95];
params = params0;
% Начальное значение ошибки
derivative0 = (simulate(params + delta*var,1) - simulate(params - delta*var,1)) / (2*delta);
derivative = derivative0;
% Количество шагов поиска коэффициентов
nSteps = 50;
    %% Параметры моделирования
    start_time = 0;
    end_time = 4;
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
    Dthetadot =[0;0;0]; 
    dW = 50; % изменение угловой скорости для придания ускорения БЛА
     %коэффициенты ПД регулятора
for i = 2:nSteps
    % подстройка параметров
     params = params - alfa * derivative;
     %расчет основной программы
     [cost,x,xdot,theta,thetadot] = simulate1(params,i,theta,thetadot,Dthetadot,theta_need,thetadot_need,x,xdot,dt,end_time,start_time);
    % расчёт градиента
    derivative = (simulate1(params + delta*var,i,theta,thetadot,Dthetadot,theta_need,thetadot_need,x,xdot,dt,end_time,start_time) - simulate1(params - delta*var,i,theta,thetadot,Dthetadot,theta_need,thetadot_need,x,xdot,dt,end_time,start_time)) / (2*delta);
end
fprintf('Начальные значения коэффициентов:\n Kp = %G, Kd = %G, Ku = %G\n', params0(1), params0(2), params0(3))
fprintf('Коэффициенты после подбора за %d итераций:\n Kp = %G, Kd = %G, Ku = %G\n', nSteps, params(1), params(2), params(3))
