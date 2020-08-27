clear;
clc
% Simulation times, in seconds.
 start_time = 0;
 end_time = 10;
 dt = 0.005;
 times = start_time:dt:end_time;
 % Number of points in the simulation.
 N = numel(times);
 % Initial simulation state.
 x = [0; 0; 10];
 xdot = zeros(3, 1);
 theta = zeros(3, 1);
 theta(2)=0;
m = 0.3;
g = 9.81;
 % Simulate some disturbance in the angular velocity.
 % The magnitude of the deviation is in radians / second.
 deviation = 100;
 thetadot = [0;0;0];%deg2rad(2 * deviation * rand(3, 1) - deviation);
%b - коэффициент крутящего момента роторов. k - подъемный коэффициент
%kd - коэффициент сопротивления среды(для силы трения)
% L - плечо от центра квадрокоптера до винтов
% I - моменты инерции, придумал сам
a = 0.245;%длина квадрокоптера
b = 0.290;%ширина квадрокоптера
h = 0.055; %высота квадрокоптера
S = zeros(3,3);%площадь квадрокоптера
S(1,1) = a*h;%Sx
S(2,2) = b*h;%Sy
S(3,3)= a*b;%Sz
r0 =1.2;%плотность воздуха
%сd - коэффициент аэродинамической силы
cd =0.4;
L = sqrt((a/2)^2+(b/2)^2);
Ix = 1/12*m*(3*L^3+h^2);
Iy = Ix;
Iz = m*L^2/2;
I =[Ix 0 0;0 Iy 0; 0 0 Iz];
b =0.7426*10^-6;
k = 1.4851*10^-5;
j=1;
W =49557.0198;% уголовая скорость вращения винтов
for t = times
 % Take input from our controller.
 %i = [340^2 340^2 340^2 340^2]; %Похоже на информацию об угловых скоростях четырех двигателей в определенный момент времениж
 i = [W W W W];
 omega = thetadot2omega(thetadot, theta);
 % Compute linear and angular accelerations.
 a = acceleration(i, theta, xdot, m, g, k, cd,S,r0);
 omegadot = angular_acceleration(i, omega, I, L, b, k);
 omega = omega + dt * omegadot;
 thetadot = omega2thetadot(omega, theta);
 theta = theta + dt * thetadot;
 xdot = xdot + dt * a;
 x = x + dt * xdot;
 X_coord(j) = x(1);
 Y_coord(j)=x(2);
 Z_coord(j)=x(3);
 Vx(j)=xdot(1);
 Vy(j)=xdot(2);
 Vz(j)=xdot(3);
 ax(j)=a(1);
 ay(j)=a(2);
 az(j)=a(3);
 j=j+1;
end
figure
hold on
plot(times,Z_coord)
plot(times,X_coord)
plot(times,Y_coord)
grid on
xlabel('Время')
ylabel('Координата')
legend('Z','X','Y')
    figure
    hold on
    plot(times,Vx)
    plot(times,Vy)
    plot(times,Vz)
    grid on
    xlabel('Время')
    ylabel('Скорость')
    legend('Vx','Vy','Vz')
figure
hold on
plot(times,ax)
plot(times,ay)
plot(times,az)
grid on
xlabel('Время')
ylabel('Ускорения')
legend('ax','ay','az')
% Step through the simulation, updating the state.