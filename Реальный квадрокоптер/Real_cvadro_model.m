clear
clc
%Конструктивные параметры
a = 0.36;%длина квадрокоптера и ширина
k = 1.4851*10^-4;% k - подъемный коэффициен
h = 0.07;%высота квадрокоптра
b = h/2;
L = 485/2/1000; %расстояние от винта квадрокоптера до цента
m = 0.3;%масса квадрокоптера
g = 9.81;%ускорение свободного падения
Ix = 1/12*m*(3*L^3+h^2);%момент инерции
Iz=Ix;%момент инерции
Iy=m*L^2/2;%момент инерции 
Irot =0;%момент инерции ротора
Sx = a*h; Sz=a*h;Sy =a*a;%площади поверхностей квадрокоптера
t =10;%Время моделирования
cd =0.1;%аэродинамический коэффициент
md =0.004;%аэродинамический коэффициент
po=1.2;%плотность воздуха
n=105/2;
% Необходимая уголовая скорость вращения винтов для зависания на месте
W1 = 128.5; W2 = 128.5; W3 = 128.5; W4 = 128.5; 
%Начальные условия по скоростям в географической и связанной СК
Vbx(1) =0; Vby(1) =0;Vbz(1) =0; Vx(1)=0; Vy(1)=0; Vz(1)=0;
%Начальные условия по угловым скоростям в географической и связанной СК
wbx(1)=0;wby(1)=0;wbz(1)=0;wx(1)=0;wy(1)=0;wz(1)=0;
%Начальные условия по углам ориентации
psi(1)=0;theta(1)=0;gamma(1)=0;
%Начальные условия по координатам
x(1)=0;y(1)=0;z(1)=0;
%Параметры для ПИД регулятора
y_need = 10;
Vy_need = 3;
sigma_need = [0,0,0];% желаемые углы ориентации
sigma_integral =[0,0,0];
%Собираем моменты инерции в один массив
I =[Ix,Iy,Iz];
dt =0.1;
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
    aty = k*(W1^2+W2^2+W3^2+W4^2);%ускорение от тяги двигателей
    ayb = agy+aky+aady+aty;
    %для оси z
    agz = g*cosd(theta(i))*sind(gamma(i));%ускорение силы тяжести на ось z
    akz = -2*(wbx(i)*Vby(i)-wby(i)*Vbx(i));%Кориолисово ускорение вдоль оси z
    aadz = -sign(Vbz(i))*cd*po*Vbz(i)^2/2/m*Sz;%ускорение, вызванное аэродинамической силой по оси z
    azb = agz+akz+aadz;
    %%угловые ускорения
    %по оси х
    dwxgyr = wby(i)*wbz(i)*((Iy-Iz)/Ix); %от собственного вращения квадрокоптера
    dwxaer = md*po*Vbx(i)^2/2/m*Sx*n;%от аэродинамического момента
    dwxeng = (k*(W2^2-W1^2)-k*(W4^2-W3^2))*L/Ix;%от вращения двигателей
    dwxrot = Irot/Ix*(W2+W1-W3-W4);%от ротора
    dwbx =dwxgyr+dwxaer+dwxeng+dwxrot;
    %по оси у
    dwygyr = wbx(i)*wbz(i)*((Iz-Ix)/Iy);%от собственного вращения квадрокоптера
    dwyaer = md*po*Vby(i)^2/2/m*Sy*n;%от аэродинамического момента
    dwyeng =(-k*(W1^2+W3^2)+k*(W2^2+W4^2))*b/Iy;%от вращения двигателей
    dwby = dwygyr+dwyaer+dwyeng;
    %по оси z
    dwzgyr = wbx(i)*wby(i)*((Ix-Iy)/Iz);%от собственного вращения квадрокоптера
    dwzaer = md*po*Vbz(i)^2/2/m*Sz*n;%от аэродинамического момента
    dwzeng = (k*(W4^2-W1^2)-k*(W2^2-W3^2))*L/Iz;%от вращения двигателей
    dwzrot = Irot/Iz*wbx(i)*(W3+W4-W2-W1);%от ротора
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
    %интеграл от угла
    sigma = [theta(i+1),gamma(i+1),psi(i+1)];
    sigmadot =[dtheta,dgamma,dpsi];
    sigma_integral = sigma_integral+sigma*dt;
    %ПИД регулятор
%     [W1,W2,W3,W4] = controller(sigma_need,sigma,sigmadot,m,g,k,0.7426*10^-4,I,L,sigma_integral);
%     [W1,W2,W3,W4] = controller_velocity(Vy(i+1),Vy_need);
[W1,W2,W3,W4] = controller_Hight(y(i+1),y_need,Vy(i+1));
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
hold on
plot(OMEGA(3,:))
hold on
plot(OMEGA(4,:))
grid on
legend('W1','W2','W3','W4')
