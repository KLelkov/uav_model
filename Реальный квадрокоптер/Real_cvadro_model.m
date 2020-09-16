clear
clc
%�������������� ���������
a = 0.36;%����� ������������� � ������
k = 1.4851*10^-4;% k - ��������� ����������
h = 0.07;%������ ������������
b = h/2;
L = 485/2/1000; %���������� �� ����� ������������� �� �����
m = 0.3;%����� �������������
g = 9.81;%��������� ���������� �������
Ix = 1/12*m*(3*L^3+h^2);%������ �������
Iz=Ix;%������ �������
Iy=m*L^2/2;%������ ������� 
Irot =0;%������ ������� ������
Sx = a*h; Sz=a*h;Sy =a*a;%������� ������������ �������������
t =10;%����� �������������
cd =0.1;%���������������� �����������
md =0.004;%���������������� �����������
po=1.2;%��������� �������
n=105/2;
W0=128.5;%������� �������� �������� ������, ��� ����������� ��������� ��� ������������
% ����������� �������� �������� �������� ������ ��� ��������� �� �����
W1 = 128.5; W2 = 128.5; W3 = 128.5; W4 = 128.5; 
%��������� ������� �� ��������� � �������������� � ��������� ��
Vbx(1) =0; Vby(1) =0;Vbz(1) =0; Vx(1)=0; Vy(1)=0; Vz(1)=0;
%��������� ������� �� ������� ��������� � �������������� � ��������� ��
wbx(1)=0;wby(1)=0;wbz(1)=0;wx(1)=0;wy(1)=0;wz(1)=0;
%��������� ������� �� ����� ����������
psi(1)=0;theta(1)=0;gamma(1)=0;
%��������� ������� �� �����������
x(1)=0;y(1)=0;z(1)=0;
%%��������� ��� ��� ����������
% % %��������� ��� ���������
% % Kp_coord = 0.4; 
% % Kd_coord = 0.7;
% % Ku_coord = 0.0;
% % %��������� ��� ���������
% % Kp_velocity = 0.05;
% % Kd_velocity = 0.0;
% % Ku_velocity = 0;
%��������� ��� ���������
Kp_y = 0; Kp_x = 0; Kp_z = 0;
Kd_y = 0; Kd_x = 0; Kd_z = 0;
Ku_y = 0; Ku_x = 0; Ku_z = 0;
%��������� ��� ���������
Kp_Vy = 0; Kp_Vx = 0; Kp_Vz = 0;
Kd_Vy = 0; Kd_Vx = 0; Kd_Vz = 0;
Ku_Vy = 0; Ku_Vx = 0; Ku_Vz = 0;
%��������� ��� �����
Kp_gamma = 1.2; Kp_theta = 0; Kp_psi = 0;
Kd_gamma = 0.09; Kd_theta = 0; Kd_psi = 0;
Ku_gamma = 0.5; Ku_theta = 0; Ku_psi = 0;
%��������� ��� ������� ���������
Kp_dgamma = 0.051;
Kd_dgamma = 0.01;
Ku_dgamma = -0.01;
%�������� ����������
y_need = 10;
x_need = 0;
z_need =0;
sigma_need = [0,0,0];% �������� ���� ����������
sigma_integral =[0,0,0];
gamma_need = 5;
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
%�������� ������� ������� � ���� ������
I =[Ix,Iy,Iz];
dt =0.1;
%%�������������
for i=1:t/dt
    %%�������� ���������
    %��� ��� �
    agx = -g*sind(theta(i)); %��������� ���� ������� �� ��� �
    akx = -2*(wby(i)*Vbz(i)-wbz(i)*Vby(i));% ����������� ��������� ����� ��� �
    aadx = -sign(Vbx(i))*cd*po*Vbx(i)^2/2/m*Sx;%���������,��������� ���������������� �����
    axb = agx+akx+aadx;%��������� ��������� ����� ��� �
   %��� ��� � 
    agy = -g*cosd(theta(i))*cosd(gamma(i));%��������� ���� ������� �� ��� �
    aky = -2*(wbz(i)*Vbx(i)-wbx(i)*Vbz(i));%����������� ��������� ����� �
    aady = -sign(Vby(i))*cd*po*Vby(i)^2/2/m*Sy;% ���������, ��������� ���������������� ����� �� �
    aty = k*(W1^2+W2^2+W3^2+W4^2);%��������� �� ���� ����������
    ayb = agy+aky+aady+aty;
    %��� ��� z
    agz = g*cosd(theta(i))*sind(gamma(i));%��������� ���� ������� �� ��� z
    akz = -2*(wbx(i)*Vby(i)-wby(i)*Vbx(i));%����������� ��������� ����� ��� z
    aadz = -sign(Vbz(i))*cd*po*Vbz(i)^2/2/m*Sz;%���������, ��������� ���������������� ����� �� ��� z
    azb = agz+akz+aadz;
    %%������� ���������
    %�� ��� �
    dwxgyr = wby(i)*wbz(i)*((Iy-Iz)/Ix); %�� ������������ �������� �������������
    dwxaer = md*po*Vbx(i)^2/2/m*Sx*n;%�� ����������������� �������
    dwxeng = (k*(W2^2+W1^2)-k*(W4^2+W3^2))*L/Ix;%�� �������� ����������
    dwxrot = Irot/Ix*wbz(i)*(W2-W1-W3+W4);%�� ������
    dwbx =dwxgyr+dwxaer+dwxeng+dwxrot;
    %�� ��� �
    dwygyr = wbx(i)*wbz(i)*((Iz-Ix)/Iy);%�� ������������ �������� �������������
    dwyaer = md*po*Vby(i)^2/2/m*Sy*n;%�� ����������������� �������
    dwyeng =(k*(W1^2+W3^2)-k*(W2^2+W4^2))*b/Iy;%�� �������� ����������
    dwby = dwygyr+dwyaer+dwyeng;
    %�� ��� z
    dwzgyr = wbx(i)*wby(i)*((Ix-Iy)/Iz);%�� ������������ �������� �������������
    dwzaer = md*po*Vbz(i)^2/2/m*Sz*n;%�� ����������������� �������
    dwzeng = (k*(W4^2+W1^2)-k*(W2^2+W3^2))*L/Iz;%�� �������� ����������
    dwzrot = Irot/Iz*wbx(i)*(W3-W4-W2+W1);%�� ������
    dwbz = dwzgyr+dwzaer+dwzeng+dwzrot;
    %���������� �������� � ������ ��������� � ��������� ��
    Vbx(i+1) = Vbx(i)+axb*dt; Vby(i+1)= Vby(i)+ayb*dt;Vbz(i+1)=Vbz(i)+azb*dt;
    wbx(i+1)= wbx(i)+dwbx*dt; wby(i+1)=wby(i)+dwby*dt; wbz(i+1)=wbz(i)+dwbz*dt;
    %�������� ��������� � ���
    dVx = axb*cosd(psi(i))*cosd(theta(i))+ayb*(-cosd(psi(i))*sind(theta(i))*cosd(gamma(i))+sind(psi(i))*sind(gamma(i)))+azb*(cosd(psi(i))*sind(theta(i))*sind(gamma(i))+sind(psi(i))+sind(psi(i))*cosd(gamma(i)));
    dVy = axb*sind(theta(i))+ayb*cosd(theta(i))*cosd(gamma(i))-azb*cosd(theta(i))*sind(gamma(i));
    dVz = -axb*sind(psi(i))*cosd(theta(i))+ayb*(cosd(psi(i))*sind(gamma(i))+sind(psi(i))*sind(theta(i))*cosd(gamma(i)))+azb*(cosd(psi(i))*cosd(gamma(i))+sind(psi(i))*sind(theta(i))*sind(gamma(i)));
    dwx = dwbx*cosd(psi(i))*cosd(theta(i))+dwby*(-cosd(psi(i))*sind(theta(i))*cosd(gamma(i))+sind(psi(i))*sind(gamma(i)))+dwbz*(cosd(psi(i))*sind(theta(i))*sind(gamma(i))+sind(psi(i))+sind(psi(i))*cosd(gamma(i)));
    dwy = dwbx*sind(theta(i))+dwby*cosd(theta(i))*cosd(gamma(i))-dwbz*cosd(theta(i))*sind(gamma(i));
    dwz = -dwbx*sind(psi(i))*cosd(theta(i))+dwby*(cosd(psi(i))*sind(gamma(i))+sind(psi(i))*sind(theta(i))*cosd(gamma(i)))+dwbz*(cosd(psi(i))*cosd(gamma(i))+sind(psi(i))*sind(theta(i))*sind(gamma(i)));
    %���������� ������� ��������� � ����� �����,����� � �������
    dpsi = (wby(i+1)*cosd(gamma(i))-wbz(i+1)*sind(gamma(i)))/cosd(theta(i));
    dtheta= wbz(i+1)*cosd(gamma(i))+wby(i+1)*sind(gamma(i));
    dgamma= wbx(i+1)-(wby(i+1)*cosd(gamma(i))-wbz(i+1)*sind(gamma(i)))*tand(theta(i));
    psi(i+1) = psi(i)+dpsi*dt; theta(i+1) =theta(i)+dtheta*dt;gamma(i+1) =gamma(i)+dgamma*dt;
    %���������� ��������� � ��������� ������������� � ���
    Vx(i+1)= Vx(i)+dVx*dt;Vy(i+1)= Vy(i)+dVy*dt;Vz(i+1)= Vz(i)+dVz*dt;
    x(i+1)= x(i)+Vx(i+1)*dt;  y(i+1)= y(i)+Vy(i+1)*dt;  z(i+1)= z(i)+Vz(i+1)*dt;
    %�������� �� ���� � ����������
    ddgamma = ddgamma+dgamma*dt;
    ddtheta = ddtheta+dtheta*dt;
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
%%��� ���������
%     [W1,W2,W3,W4] = controller(sigma_need,sigma,sigmadot,m,g,k,0.7426*10^-4,I,L,sigma_integral);
%     [W1,W2,W3,W4] = controller_velocity(Vy(i+1),Vy_need);
  %[W1,W2,W3,W4] = controller_Hight(y(i+1),y_need,Vy(i+1));
  %��� ������������� ������
  Vy_need = Kp_y*(y_need-y(i+1))+Kd_y*Vy(i+1)+Ku_y*y_integral;%�������� ������������ ��������
  delta_Fy = Kp_Vy*(Vy_need-Vy(i+1))+Kd_Vy*dVy + Ku_Vy*y(i+1);
  gamma_dot_need = Kp_gamma*(gamma_need-gamma(i+1))+Kd_gamma*dgamma + Ku_gamma*gamma_integral;%�������� ������� �������� �� gamma
  delta_Mx = Kp_dgamma*(gamma_dot_need-dgamma)+Kd_dgamma*ddgamma + Ku_dgamma*gamma(i+1);
  %%��� ���������������(��� � � z)
  %��� �
  Vx_need = Kp_x*(x_need-x(i+1))+Kd_x*Vx(i+1)+Ku_x*x_integral;
  theta_need = Kp_Vx*(Vx_need -Vx(i+1))+Kd_Vx*dVx +Ku_Vx*x(i+1);
  delta_Mz = Kp_theta*(theta_need - theta(i+1))+Kd_theta*dtheta+Ku_theta*theta_integral;
  % ��� z
  Vz_need = Kp_z*(z_need-z(i+1))+Kd_z*Vz(i+1)+Ku_z*z_integral;
  psi_need = Kp_Vz*(Vz_need -Vz(i+1))+Kd_Vz*dVz +Ku_Vz*z(i+1);
  delta_My = Kp_psi*(psi_need - psi(i+1))+Kd_psi*dpsi+Ku_psi*psi_integral;
  %������������ ��������� �� ���������
  W1 = W0+(Iy*L^2*delta_Fy + Ix^2*b*delta_Mx + Ix*Iy*L*delta_Mx + 2*Ix*Iy*L*delta_My + 2*Iy*Iz*L*delta_Mz + Ix*L*b*delta_Fy)/(4*L*k*(Ix*b + Iy*L));
  W2 = W0+(Iy*L^2*delta_Fy + Ix^2*b*delta_Mx + Ix*Iy*L*delta_Mx - 2*Ix*Iy*L*delta_My - 2*Iy*Iz*L*delta_Mz + Ix*L*b*delta_Fy)/(4*L*k*(Ix*b + Iy*L));
  W3 = W0+(Iy*L^2*delta_Fy - Ix^2*b*delta_Mx - Ix*Iy*L*delta_Mx + 2*Ix*Iy*L*delta_My - 2*Ix*Iz*b*delta_Mz + Ix*L*b*delta_Fy)/(4*L*k*(Ix*b + Iy*L));
  W4 = W0+(Iy*L^2*delta_Fy - Ix^2*b*delta_Mx - Ix*Iy*L*delta_Mx - 2*Ix*Iy*L*delta_My + 2*Ix*Iz*b*delta_Mz + Ix*L*b*delta_Fy)/(4*L*k*(Ix*b + Iy*L));
OMEGA(:,i)=[W1,W2,W3,W4]; 
end
%%���������� ��������
%��� ���������
figure
plot(x)
hold on 
plot(y)
hold on
plot(z)
grid on
legend('x','y','z')
xlabel('�����')
ylabel('����������')
%��� ���������
figure
plot(Vx)
hold on
plot(Vy)
hold on
plot(Vz)
grid on
legend('Vx','Vy','Vz')
xlabel('�����')
ylabel('��������')
%��� ����� ����������
figure
plot(psi)
hold on
plot(theta)
hold on
plot(gamma)
grid on
legend('psi','theta','gamma')
xlabel('�����')
ylabel('���� ����������')
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
