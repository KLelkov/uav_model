clear;
clc
%% ��������� �������������
start_time = 0;
end_time = 30;
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
%% �������������� ���������
m = 0.3;
a = 0.245;%����� �������������
width = 0.290;%������ �������������
h = 0.055; %������ �������������
S = zeros(3,3);%������� �������������
S(1) = a*h;%Sx
S(2) = width*h;%Sy
S(3)= width*a;%Sz
% L - ����� �� ������ ������������� �� ������
L = sqrt((a/2)^2+(width/2)^2);
Ix = 1/12*m*(3*L^3+h^2);
Iy = Ix;
Iz = m*L^2/2;
% I - ������� �������, �������� ���
I =[Ix 0 0;0 Iy 0; 0 0 Iz];
%% ���������� ��������
g = 9.81;
r0 =1.2;%��������� �������
%�d - ����������� ���������������� ����
cd =0.4;
%b - ����������� ��������� ������� �������. k - ��������� �����������
b =0.7426*10^-6;
k = 1.4851*10^-5;
% ����������� �������� �������� �������� ������ ��� ��������� �� �����
W_hover = 49257.0198; % ������� ������� ��������
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
dW = 50; % ��������� ������� �������� ��� �������� ��������� ���
% �������� �������� ������
%i = [W_hover+dW, W_hover+dW, W_hover+dW, W_hover+dW];
%% �������� ����
for j = 1:nSim
    % ������ ������� ��������� � ��������� (body) ��
    i = PD_controller(theta_need,theta,thetadot_need,thetadot,m,g,k,b,I,L);
    omega = thetadot2omega(thetadot, theta);
    % ������ �������� ���������
    a = acceleration(i, theta, xdot, m, g, k, cd,S,r0);
    % ������ ������� ���������
    omegadot = angular_acceleration(i, omega, I, L, b, k);
    % �������������� ������� ���������
    omega = omega + dt * omegadot;
    % ������ ������� ��������� � ������������ (inertial) ��
    thetadot = omega2thetadot(omega, theta);
    % ������ ����� ����������
    theta = theta + dt * thetadot;
    % ������ ��������� ��������
    xdot = xdot + dt * a;
    % ������ ���������
    x = x + dt * xdot;
    % ���� ����� ��� ��������
    Coords(j,:) = x(:,1);
    Speed(j,:) = xdot(:,1);
    Acceleration(j,:) = a(:,1);
    Angles(j,:) = theta;
    Rates(j,:) = thetadot;
    Time(j) = j*dt;
end
%% ���������� ��������
close all
figure('Name', '����������')
plot(Time, Coords, 'LineWidth', 2);
grid on
xlabel('�����, �')
ylabel('����������, �')
legend X Y Z
title ����������

figure('Name', '�������� � ���������')
subplot(2,1,1)
plot(Time, Speed, 'LineWidth', 2);
grid on
xlabel('�����, �')
ylabel('��������, �/�')
legend Vx Vy Vz
title '��������'
subplot(2,1,2)
plot(Time, Acceleration, 'LineWidth', 2);
grid on
xlabel('�����, �')
ylabel('���������, �/�2')
legend ax ay az
title '���������'
figure('Name', '���� � ������� ��������')
subplot(2,1,1)
plot(Time, Angles, 'LineWidth', 2);
grid on
xlabel('�����, �')
ylabel('����, ����')
legend ���� ������ ����
title '���� ����������'
subplot(2,1,2)
plot(Time, Rates, 'LineWidth', 2);
grid on
xlabel('�����, �')
ylabel('������� ��������, ???')
legend wx wy wz
title '������� ��������'


