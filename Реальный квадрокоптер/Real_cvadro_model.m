clear
clc
close all
%Конструктивные параметры
a = 0.36;%длина квадрокоптера и ширина
k = 1.4851*10^-4;% k - подъемный коэффициен
h = 0.07;%высота квадрокоптра
b = h/2;
L = 485/2/1000; %расстояние от винта квадрокоптера до цента
% Когда винты располагаются схема Х, плечо моторов не равно длине плеча L
L = 0.36/2;
m = 0.7;%масса квадрокоптера
g = 9.81;%ускорение свободного падения
Ix = 1/12*m*(3*L^3+h^2);%момент инерции
Iz=Ix;%момент инерции
Iy=m*L^2/2;%момент инерции 
Irot =0;%момент инерции ротора
Sx = a*h; Sz=a*h;Sy =a*a;%площади поверхностей квадрокоптера
t =25;%Время моделирования
cd =0.1;%аэродинамический коэффициент
md =0.004;%аэродинамический коэффициент
po=1.2;%плотность воздуха
n=105/2;
W0=107.6;%угловая скорость вращения винтов, для обеспечения зависания над поверхностью
% Необходимая уголовая скорость вращения винтов для зависания на месте
W1 = W0; W2 = W0; W3 = W0; W4 = W0;
Wmin = 50;
Wmax = 400;
dWmax = Wmax - W1;
dWmin = Wmin - W1;
%Начальные условия по скоростям в географической и связанной СК
Vbx(1) =0; Vby(1) =0;Vbz(1) =0; Vx(1)=0; Vy(1)=0; Vz(1)=0;
%Начальные условия по угловым скоростям в географической и связанной СК
wbx(1)=0;wby(1)=0;wbz(1)=0;wx(1)=0;wy(1)=0;wz(1)=0;
%Начальные условия по углам ориентации
psi(1)=0;theta(1)=0;gamma(1)=0;
%Начальные условия по координатам
x(1)=0;y(1)=0;z(1)=0;
%%Параметры для ПИД регулятора
%регулятор для координат
Kp_y = 0.8; Kp_x = 0.004; Kp_z = 0;%Kp_y = 0.4
Kd_y = 0; Kd_x = 2; Kd_z = 0;%Kd_y = 0.7
Ku_y = 0; Ku_x = 1; Ku_z = 0;
%регулятор для скоростей
Kp_Vy = 0.05; Kp_Vx = 0.000001; Kp_Vz = 0;%Kp_Vy =0.05
Kd_Vy = 0; Kd_Vx = 0; Kd_Vz = 0;
Ku_Vy = 0; Ku_Vx = -0; Ku_Vz = 0;
%регулятор для углов
Kp_gamma = 0; Kp_theta = 0.000001; Kp_psi = 0;
Kd_gamma = 0.0; Kd_theta = 0; Kd_psi = 0;
Ku_gamma = 0.0; Ku_theta = -0; Ku_psi = 0;
%регулятор для угловых скоростей
Kp_dgamma = 0.051;
Kd_dgamma = 0.01;
Ku_dgamma = -0.01;
%желаемая координата
y_need = 0;
x_need = 0;
z_need =0;
sigma_need = [0,0,0];% желаемые углы ориентации
sigma_integral =[0,0,0];
gamma_need = 0;
y_integral = 0;
x_integral = 0;
z_integral = 0;
gamma_integral = 0;
ddgamma = 0;
theta_need = 0;
psi_need = 0;
theta_integral = 0;
psi_integral = 0;
ddtheta = 0;
ddpsi = 0;
%Собираем моменты инерции в один массив
I =[Ix,Iy,Iz];
dt =0.1;
%%Моделирование
md = 0;
dtheta_last = 0;
for i=1:t/dt
    %%линейные ускорения
    %для оси х
    agx = -g*sind(theta(i)); %ускорение силы тяжести на ось х
    akx = -2*(wby(i)*Vbz(i)-wbz(i)*Vby(i));% Кориолисово ускорение вдоль оси х
    aadx = -sign(Vbx(i))*cd*po*Vbx(i)^2/2/m*Sx;%ускорения,вызванное аэродинамической силой
    axb = agx+akx+aadx;%суммарное ускорение вдоль оси х
   %для оси у 
    agy = -g*cosd(theta(i))*cosd(gamma(i));%ускорение силы тяжести на ось у
    aky = -2*(wbz(i)*Vbx(i)-wbx(i)*Vbz(i));%Кориолисово ускорение вдоль у
    aady = -sign(Vby(i))*cd*po*Vby(i)^2/2/m*Sy;% ускорение, вызванное аэродинамической силой по у
    aty = k*(W1^2+W2^2+W3^2+W4^2) / m;%ускорение от тяги двигателей
    ayb = agy+aky+aady+aty;
    %для оси z
    agz = g*cosd(theta(i))*sind(gamma(i));%ускорение силы тяжести на ось z
    akz = -2*(wbx(i)*Vby(i)-wby(i)*Vbx(i));%Кориолисово ускорение вдоль оси z
    aadz = -sign(Vbz(i))*cd*po*Vbz(i)^2/2/m*Sz;%ускорение, вызванное аэродинамической силой по оси z
    azb = agz+akz+aadz;
    %%угловые ускорения
    %по оси х
    dwxgyr = wby(i)*wbz(i)*((Iy-Iz)/Ix); %от собственного вращения квадрокоптера
    dwxaer = md*po*Vbx(i)^2/2*Sx*n;%от аэродинамического момента
    dwxeng = (k*(W2^2+W1^2)-k*(W4^2+W3^2))*L/Ix;%от вращения двигателей
    dwxrot = Irot/Ix*wbz(i)*(W2-W1-W3+W4);%от ротора
    dwbx =dwxgyr+dwxaer+dwxeng+dwxrot;
    %по оси у
    dwygyr = wbx(i)*wbz(i)*((Iz-Ix)/Iy);%от собственного вращения квадрокоптера
    dwyaer = md*po*Vby(i)^2/2*Sy*n;%от аэродинамического момента
    dwyeng =(k*(W1^2+W3^2)-k*(W2^2+W4^2))*b/Iy;%от вращения двигателей
    dwby = dwygyr+dwyaer+dwyeng;
    %по оси z
    dwzgyr = wbx(i)*wby(i)*((Ix-Iy)/Iz);%от собственного вращения квадрокоптера
    dwzaer = md*po*Vbz(i)^2/2*Sz*n;%от аэродинамического момента
    dwzeng = (k*(W4^2+W1^2)-k*(W2^2+W3^2))*L/Iz;%от вращения двигателей
    dwzrot = Irot/Iz*wbx(i)*(W3-W4-W2+W1);%от ротора
    dwbz = dwzgyr+dwzaer+dwzeng+dwzrot;
    %вычисление линейных и гловых скоростей в связанной СК
    Vbx(i+1) = Vbx(i)+axb*dt; Vby(i+1)= Vby(i)+ayb*dt;Vbz(i+1)=Vbz(i)+azb*dt;
    wbx(i+1)= wbx(i)+dwbx*dt; wby(i+1)=wby(i)+dwby*dt; wbz(i+1)=wbz(i)+dwbz*dt;
    %пересчет ускорений в ИСК
    dVx = axb*cosd(psi(i))*cosd(theta(i))+ayb*(-cosd(psi(i))*sind(theta(i))*cosd(gamma(i))+sind(psi(i))*sind(gamma(i)))+azb*(cosd(psi(i))*sind(theta(i))*sind(gamma(i))+sind(psi(i))+sind(psi(i))*cosd(gamma(i)));
    dVy = axb*sind(theta(i))+ayb*cosd(theta(i))*cosd(gamma(i))-azb*cosd(theta(i))*sind(gamma(i));
    dVz = -axb*sind(psi(i))*cosd(theta(i))+ayb*(cosd(psi(i))*sind(gamma(i))+sind(psi(i))*sind(theta(i))*cosd(gamma(i)))+azb*(cosd(psi(i))*cosd(gamma(i))+sind(psi(i))*sind(theta(i))*sind(gamma(i)));
    dwx = dwbx*cosd(psi(i))*cosd(theta(i))+dwby*(-cosd(psi(i))*sind(theta(i))*cosd(gamma(i))+sind(psi(i))*sind(gamma(i)))+dwbz*(cosd(psi(i))*sind(theta(i))*sind(gamma(i))+sind(psi(i))+sind(psi(i))*cosd(gamma(i)));
    dwy = dwbx*sind(theta(i))+dwby*cosd(theta(i))*cosd(gamma(i))-dwbz*cosd(theta(i))*sind(gamma(i));
    dwz = -dwbx*sind(psi(i))*cosd(theta(i))+dwby*(cosd(psi(i))*sind(gamma(i))+sind(psi(i))*sind(theta(i))*cosd(gamma(i)))+dwbz*(cosd(psi(i))*cosd(gamma(i))+sind(psi(i))*sind(theta(i))*sind(gamma(i)));
    %вычисление угловых скоростей и углов курса,крена и тангажа
    dpsi = (wby(i+1)*cosd(gamma(i))-wbz(i+1)*sind(gamma(i)))/cosd(theta(i));
    dtheta= wbz(i+1)*cosd(gamma(i))+wby(i+1)*sind(gamma(i));
    dgamma= wbx(i+1)-(wby(i+1)*cosd(gamma(i))-wbz(i+1)*sind(gamma(i)))*tand(theta(i));
    psi(i+1) = psi(i)+dpsi*dt; theta(i+1) =theta(i)+dtheta*dt;gamma(i+1) =gamma(i)+dgamma*dt;
    %вычисление скоростей и координат квадрокоптера в ИСК
    Vx(i+1)= Vx(i)+dVx*dt;Vy(i+1)= Vy(i)+dVy*dt;Vz(i+1)= Vz(i)+dVz*dt;
    x(i+1)= x(i)+Vx(i+1)*dt;  y(i+1)= y(i)+Vy(i+1)*dt;  z(i+1)= z(i)+Vz(i+1)*dt;
    %интеграл от угла и координаты
    ddgamma = ddgamma+dgamma*dt;
    ddtheta = (dtheta - dtheta_last) / dt;
    dtheta_last = dtheta;
    ddpsi = ddpsi+dpsi*dt;
    sigma = [theta(i+1),gamma(i+1),psi(i+1)];
    sigmadot =[dtheta,dgamma,dpsi];
    sigma_integral = sigma_integral+sigma*dt;
    y_integral = y_integral+y(i+1)*dt;
    x_integral = x_integral+x(i+1)*dt;
    z_integral = z_integral+z(i+1)*dt;
    gamma_integral = gamma_integral+gamma(i+1)*dt;
    theta_integral = theta_integral+theta(i+1)*dt;
    psi_integral = psi_integral+psi(i+1)*dt;
%%ПИД регулятор
%     [W1,W2,W3,W4] = controller(sigma_need,sigma,sigmadot,m,g,k,0.7426*10^-4,I,L,sigma_integral);
%     [W1,W2,W3,W4] = controller_velocity(Vy(i+1),Vy_need);
  %[W1,W2,W3,W4] = controller_Hight(y(i+1),y_need,Vy(i+1));
  % --- ВЕРТИКАЛЬНЫЙ КАНАЛ ---
  y_need = 0;
  Vy_need = 0.8*(y_need-y(i+1))-0.9*Vy(i+1)-0.0*y_integral;%желаемая вертикальная скорость
  delta_Fy = Kp_Vy*(Vy_need-Vy(i+1))+Kd_Vy*dVy + Ku_Vy*y(i+1);
  % Это условие убирает шум управляющего сигнала. В любом случае,
  % практическая реализация изменения момента на 0.005 Нм невозможна.
  if abs(delta_Fy) < 0.003, delta_Fy = 0; end 
%   debug(i,1) = Vy_need;
%   debug(i,2) = y_need-y(i+1);
%   debug(i,3) = delta_Fy;
  % ---
  gamma_dot_need = Kp_gamma*(gamma_need-gamma(i+1))+Kd_gamma*dgamma + Ku_gamma*gamma_integral;%желаемая угловая скорость по gamma
  delta_Mx = Kp_dgamma*(gamma_dot_need-dgamma)+Kd_dgamma*ddgamma + Ku_dgamma*gamma(i+1);
  %%для горизонтального(оси х и z)
  %ось х
  % --- КАНАЛ по оси Х ---
  % TODO: Подобрать коэффициенты
  x_need = 40;
   Vx_need = -0.1*(x_need-x(i+1))+0*Vx(i+1)+0*x_integral;
    Vx_need = 5;
  theta_need = -0.02*(Vx_need - Vx(i+1)) + (-0.00)*dVx + 0*x(i+1);
  theta_need = 10;
  
  % --- КАНАЛ ТАНГАЖА ---
  % TODO: улучшить качество переходного процесса
  theta_need = min(max(theta_need, -10), 10);
  theta_dot_need = 0.5*(theta_need - theta(i+1)) - 0.2*dtheta + 0*theta_integral;
  theta_dot_need = 1.5;
  delta_Mz = 0.0001*(theta_dot_need - dtheta) - 0.0001*ddtheta + 0*theta(i+1);
  debug(i,1) = dtheta;
  debug(i,2) = dwzeng;
  debug(i,3) = delta_Mz;
  % ---
  % ось z
  Vz_need = Kp_z*(z_need-z(i+1))+Kd_z*Vz(i+1)+Ku_z*z_integral;
  psi_need = Kp_Vz*(Vz_need -Vz(i+1))+Kd_Vz*dVz +Ku_Vz*z(i+1);
  delta_My = Kp_psi*(psi_need - psi(i+1))+Kd_psi*dpsi+Ku_psi*psi_integral;
  %формирование мощностей на двигатели
  % Решением системы уравнений являются квадраты изменения скорости
  % вращения моторов
  dW1 = (L*delta_My + b*delta_Mx + b*delta_Mz + L*b*delta_Fy) / (4*L*b*k);
  dW2 = -(L*delta_My - b*delta_Mx + b*delta_Mz - L*b*delta_Fy) / (4*L*b*k);
  dW3 = (L*delta_My - b*delta_Mx - b*delta_Mz + L*b*delta_Fy) / (4*L*b*k);
  dW4 = -(L*delta_My + b*delta_Mx - b*delta_Mz - L*b*delta_Fy) / (4*L*b*k);
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
  W1 = W0+dW1;
  W2 = W0+dW2;
  W3 = W0+dW3;
  W4 = W0+dW4;
OMEGA(:,i)=[W1,W2,W3,W4]; 
end
%%построение графиков
%для координат
figure
plot(x)
hold on 
plot(y)
hold on
plot(z)
grid on
legend('x','y','z')
xlabel('Время')
ylabel('Координаты')
%для скоростей
figure
plot(Vx)
hold on
plot(Vy)
hold on
plot(Vz)
grid on
legend('Vx','Vy','Vz')
xlabel('Время')
ylabel('Скорости')
%для углов ориентации
figure
plot(psi)
hold on
plot(theta)
hold on
plot(gamma)
grid on
legend('psi','theta','gamma')
xlabel('Время')
ylabel('углы ориентации')
figure
plot(OMEGA(1,:))
hold on
plot(OMEGA(2,:))
plot(OMEGA(3,:))
plot(OMEGA(4,:))
grid on
legend('W1','W2','W3','W4')

figure
plot(debug)
grid on
