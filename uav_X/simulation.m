clear
clc
close all

%% ��������� �������
% ���������� � ������������ ��
uav_state.coords = [0; 0; 0];
% ���� ����������
uav_state.roll = 0;
uav_state.pitch = 0;
uav_state.yaw = 30;
% �������� �������� � ������������ ��
uav_state.V = [0; 0; 0];
% ������� �������� � ������������ ��
uav_state.W = [0; 0; 0];
roll = uav_state.roll; pitch = uav_state.pitch; yaw = uav_state.yaw;
% ������� �������� �� ��������� �� � ������������ ��
Rbg = [cosd(yaw)*cosd(pitch), -cosd(yaw)*sind(pitch)*cosd(roll)+sind(yaw)*sind(roll), cosd(yaw)*sind(pitch)*sind(roll)+sind(yaw)*cosd(roll);
    sind(pitch), cosd(pitch)*cosd(roll), -cosd(pitch)*sind(roll);
    -sind(yaw)*cosd(pitch), cosd(yaw)*sind(roll)+sind(yaw)*sind(pitch)*cosd(roll), cosd(yaw)*cosd(roll)-sind(yaw)*sind(pitch)*sind(roll)];
% �������� �������� � ��������� ��
uav_state.Vb = Rbg' * uav_state.V;
% ������� �������� � ��������� ��
uav_state.Wb = [0; 0; 0];
uav_state.droll = uav_state.W(1); % ������� �������� ������ ��� X
uav_state.dpitch = uav_state.W(3); % ������� �������� ������ ��� Z
uav_state.dyaw = uav_state.W(2); % ������� �������� ������ ��� Y

W0 = 107.5;

%% ������� �������
% ����������� ���������� (����������� ������ ���������� �� Y)
flight_mission.coords = [0; 0; 0];
% ����������� �������� � �������������� ��: ����������, ������������ �
% �������
flight_mission.V = [5; 0; 0];
% ���������� ���� ���������� (���� ����� � ������� ��������������
% ������������� � ����������� �� ��������)
flight_mission.roll = 0;
flight_mission.pitch = 0;
flight_mission.yaw = 30;
% ������������ ������� �������� (�������������� �������������)
flight_mission.droll = 0;
flight_mission.dpitch = 0;
flight_mission.dyaw = 0;

%% ��������� �������������
simTime = 20; % seconds
dt = 0.01; % simulation step
nSim = simTime / dt;

%% �������� ��������
V = zeros(nSim, 3);
W = zeros(nSim, 3);
Vb = zeros(nSim, 3);
Wb = zeros(nSim, 3);
angles = zeros(nSim, 3);
coords = zeros(nSim, 3);
rates = zeros(nSim, 3);
Time = zeros(nSim, 1);
motors = zeros(nSim, 4);

V(1,:) = uav_state.V';
W(1,:) = uav_state.W';
Vb(1,:) = uav_state.Vb';
Wb(1,:) = uav_state.Wb';
angles(1,:) = [uav_state.roll, uav_state.pitch, uav_state.yaw];
rates(1,:) = uav_state.W';
coords(1,:) = uav_state.coords';
motors(1,:) = [W0, W0, W0, W0];

%% ��������� ������ ��� ����������� ���������� ���������
reg_state.W0 = W0;
reg_state.dt = dt;
% ����� �������
reg_state.pitch.D = 0;
reg_state.pitch.I = 0;
% ����� �����
reg_state.roll.D = 0;
reg_state.roll.I = 0;
% ����� �����
reg_state.yaw.D = 0;
reg_state.yaw.I = 0;
reg_state.dyaw.D = 0;
reg_state.dyaw.I = 0;
% ����� ������������ ��������
reg_state.vy.D = 0;
reg_state.vy.I = 0;
% ����� ������� ��������
reg_state.vr.D = 0;
reg_state.vr.I = 0;
% ����� ���������� ��������
reg_state.vf.D = 0;
reg_state.vf.I = 0;

%% �������� ����
for i = 2:nSim
    
    % ������ ������������ ����������
    [controls, reg_state] = uav_control(uav_state, flight_mission, reg_state);
    % ����� �������������� ������
    % (���������� �������� ���������� �� ��������� ������� dt)
    uav_state = uav_model_X(uav_state, controls, dt);
    
    % ���� ����� ��� �������� � �������������
    V(i,:) = uav_state.V';
    W(i,:) = uav_state.W';
    Vb(i,:) = uav_state.Vb';
    Wb(i,:) = uav_state.Wb';
    angles(i,:) = [uav_state.roll, uav_state.pitch, uav_state.yaw];
    coords(i,:) = uav_state.coords';
    rates(i,:) = [uav_state.droll, uav_state.dpitch, uav_state.dyaw];
    motors(i,:) = controls;
    Time(i) = Time(i-1) + dt;
    
end

%% ���������� � ����
figure;
subplot(2,1,1);
plot(Time, coords);
grid on;
xlabel 'Time, s'
ylabel 'Coordinate, m'
title 'Coordinates'
legend x y z

subplot(2,1,2);
plot(Time, angles);
grid on;
xlabel 'Time, s'
ylabel 'Angle, deg'
title 'Orientation'
legend roll pitch yaw

%% �������� ��������
figure;
subplot(2,1,1);
plot(Time, Vb);
grid on;
xlabel 'Time, s'
ylabel 'Velocity, m/s'
title 'Velocity in base frame'
legend Vbx Vby Vbz

subplot(2,1,2);
plot(Time, V);
grid on;
xlabel 'Time, s'
ylabel 'Velocity, m/s'
title 'Velocity in inertia frame'
legend Vx Vy Vz

%% ������� ��������
figure;
subplot(3,1,1);
plot(Time, Wb);
grid on;
xlabel 'Time, s'
ylabel 'Rate, rad/s'
title 'Angular rate in base frame'
legend Wbx Wby Wbz

subplot(3,1,2);
plot(Time, W);
grid on;
xlabel 'Time, s'
ylabel 'Rate, rad/s'
title 'Angular rate in inertia frame'
legend Wx Wy Wz

subplot(3,1,3);
plot(Time, rates);
grid on;
xlabel 'Time, s'
ylabel 'Rate, rad/s'
title 'Orientation rates'
legend droll dpitch dyaw

%% ���� �������
figure;
plot(Time, motors);
grid on; hold on
plot([Time(1), Time(end)], [W0 W0], '--k');
xlabel 'Time, s'
ylabel 'Rotation speed, rad/s'
title 'Thrust'
legend W1 W2 W3 W4