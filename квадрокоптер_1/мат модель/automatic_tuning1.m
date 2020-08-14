clear;
clc;
% ��������� ����������
delta = 0.8;
alfa = 1.2;
% ������ ������������ ������ �������������
var = [0.7, 0.3, 0]; %Uk
% ��������� ������������� ��������� �������������
params0 = [rand()*3, rand()*3, rand()*2];
% params0 = [2, 1.4, 0.95];
params = params0;
% ��������� �������� ������
derivative0 = (simulate(params + delta*var,1) - simulate(params - delta*var,1)) / (2*delta);
derivative = derivative0;
% ���������� ����� ������ �������������
nSteps = 50;
    %% ��������� �������������
    start_time = 0;
    end_time = 4;
    dt = 0.005; % ��� �������������
    nSim = (end_time - start_time)/ dt; % ���������� ����� ��� �������������
    times = start_time:dt:end_time;
    % ������� ��� ����� ����� � ���������� ��������
    Coords = zeros(nSim, 3);
    Speed = zeros(nSim, 3);
    Acceleration = zeros(nSim, 3);
    Angles = zeros(nSim, 3);
    Rates = zeros(nSim, 3);
    Time = zeros(nSim, 1);
    %% ������������� �����
    % Simulate some disturbance in the angular velocity.
    % The magnitude of the deviation is in radians / second.
    % deviation = 100;
    % thetadot = deg2rad(2 * deviation * rand(3, 1) - deviation);

     %% ��������� �������
    x = [0; 0; 10]; % X Y Z
    xdot = [0; 0; 0]; % Vx Vy Vz
    theta = [5; 0; 0]; % ���� ������ ����
    thetadot = [0;0;0];
    theta_need =[0;0;0];% �������� ����, ������,����
    thetadot_need=[0;0;0];%�������� ������� ��������
    Dthetadot =[0;0;0]; 
    dW = 50; % ��������� ������� �������� ��� �������� ��������� ���
     %������������ �� ����������
for i = 2:nSteps
    % ���������� ����������
     params = params - alfa * derivative;
     %������ �������� ���������
     [cost,x,xdot,theta,thetadot] = simulate1(params,i,theta,thetadot,Dthetadot,theta_need,thetadot_need,x,xdot,dt,end_time,start_time);
    % ������ ���������
    derivative = (simulate1(params + delta*var,i,theta,thetadot,Dthetadot,theta_need,thetadot_need,x,xdot,dt,end_time,start_time) - simulate1(params - delta*var,i,theta,thetadot,Dthetadot,theta_need,thetadot_need,x,xdot,dt,end_time,start_time)) / (2*delta);
end
fprintf('��������� �������� �������������:\n Kp = %G, Kd = %G, Ku = %G\n', params0(1), params0(2), params0(3))
fprintf('������������ ����� ������� �� %d ��������:\n Kp = %G, Kd = %G, Ku = %G\n', nSteps, params(1), params(2), params(3))
