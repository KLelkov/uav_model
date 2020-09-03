clear
clc
%�������������� ���������
a = 0.36;%����� ������������� � ������
k = 1.4851*10^-4;% k - ��������� ����������
h = 0.07;%������ ������������
b = h/2;
L = 485/2; %���������� �� ����� ������������� �� �����
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
% ����������� �������� �������� �������� ������ ��� ��������� �� �����
W1 = 128.5; W2 = 128.5; W3 = 128.5; W4 = 128.5; 
%��������� ������� �� ��������� � �������������� � ��������� ��
Vbx(1) =0; Vby(1) =0;Vbz(1) =0;Vx(1)=0;Vy(1)=0;Vz(1)=0;
%��������� ������� �� ������� ��������� � �������������� � ��������� ��
wbx(1)=0;wby(1)=0;wbz(1)=0;wx(1)=0;wy(1)=0;wz(1)=0;
%��������� ������� �� ����� ����������
psi(1)=5;theta(1)=0;gamma(1)=0;
%��������� ������� �� �����������
x(1)=0;y(1)=0;z(1)=0;
%��������� ��� ��� ����������
psi_need =0;% �������� ���� �����
psi_integral =0;
for i=1:t
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
    dwxeng = (k*(W2^2-W1^2)-k*(W4^2-W3^2))*L/Ix;%�� �������� ����������
    dwxrot = Irot/Ix*(W2+W1-W3-W4);%�� ������
    dwbx =dwxgyr+dwxaer+dwxeng+dwxrot;
    %�� ��� �
    dwygyr = wbx(i)*wbz(i)*((Iz-Ix)/Iy);%�� ������������ �������� �������������
    dwyaer = md*po*Vby(i)^2/2/m*Sy*n;%�� ����������������� �������
    dwyeng =(-k*(W1^2+W3^2)+k*(W2^2+W4^2))*b/Iy;%�� �������� ����������
    dwby = dwygyr+dwyaer+dwyeng;
    %�� ��� z
    dwzgyr = wbx(i)*wby(i)*((Ix-Iy)/Iz);%�� ������������ �������� �������������
    dwzaer = md*po*Vbz(i)^2/2/m*Sz*n;%�� ����������������� �������
    dwzeng = (k*(W4^2-W1^2)-k*(W2^2-W3^2))*L/Iz;%�� �������� ����������
    dwzrot = Irot/Iz*wbx(i)*(W3+W4-W2-W1);%�� ������
    dwbz = dwzgyr+dwzaer+dwzeng+dwzrot;
    %���������� �������� � ������ ��������� � ��������� ��
    Vbx(i+1) = Vbx(i)+axb; Vby(i+1)= Vby(i)+ayb;Vbz(i+1)=Vbz(i)+azb;
    wbx(i+1)= wbx(i)+dwbx; wby(i+1)=wby(i)+dwby; wbz(i+1)=wbz(i)+dwbz;
    %�������� ��������� � ���
    dVx = axb*cosd(psi(i))*cosd(theta(i))+ayb*(-cosd(psi(i))*sind(theta(i))*cosd(gamma(i))+sind(psi(i))*sind(gamma(i)))+azb*(cosd(psi(i))*sind(theta(i))*sind(gamma(i))+sind(psi(i))+sind(psi(i))*cosd(gamma(i)));
    dVy = axb*sind(theta(i))+ayb*cosd(theta(i))*cosd(gamma(i))-azb*cosd(theta(i))*sind(gamma(i));
    dVz = -axb*sind(psi(i))*cosd(theta(i))+ayb*(cosd(psi(i))*sind(gamma(i))+sind(psi(i))*sind(theta(i))*cosd(gamma(i)))+azb*(cosd(psi(i))*cosd(gamma(i))+sind(psi(i))*sind(theta(i))*sind(gamma(i)));
    dwx = dwbx*cosd(psi(i))*cosd(theta(i))+dwby*(-cosd(psi(i))*sind(theta(i))*cosd(gamma(i))+sind(psi(i))*sind(gamma(i)))+dwbz*(cosd(psi(i))*sind(theta(i))*sind(gamma(i))+sind(psi(i))+sind(psi(i))*cosd(gamma(i)));
    dwy = dwbx*sind(theta(i))+dwby*cosd(theta(i))*cosd(gamma(i))-dwbz*cosd(theta(i))*sind(gamma(i));
    dwz = -dwbx*sind(psi(i))*cosd(theta(i))+dwby*(cosd(psi(i))*sind(gamma(i))+sind(psi(i))*sind(theta(i))*cosd(gamma(i)))+dwbz*(cosd(psi(i))*cosd(gamma(i))+sind(psi(i))*sind(theta(i))*sind(gamma(i)));
    %���������� ������� ��������� � ����� �����,����� � �������
    dpsi = (wby(i+1)*cosd(gamma(i))-wbz(i+1)*sind(gamma(i)))/cosd(theta(i));
    dtheta=wbz(i+1)*cosd(gamma(i))+wby(i+1)*sind(gamma(i));
    dgamma=wbx(i+1)-(wby(i+1)*cosd(gamma(i))-wbz(i+1)*sind(gamma(i)))*tand(theta(i));
    psi(i+1) =psi(i)+dpsi; theta(i+1) =theta(i)+dtheta;gamma(i+1) =gamma(i)+dgamma;
    %���������� ��������� � ��������� ������������� � ���
    Vx(i+1)= Vx(i)+dVx;Vy(i+1)= Vy(i)+dVy;Vz(i+1)= Vz(i)+dVz;
    x(i+1)= x(i)+Vx(i+1);  y(i+1)= y(i)+Vy(i+1);  z(i+1)= z(i)+Vz(i+1);
    %�������� �� ����
    psi_integral = psi_integral+psi(i+1);
    %��� ���������
    [W1,W2,W3,W4] = controller(psi(i+1),theta(i+1),gamma(i+1),psi_need,dpsi,m,g,k,0.7426*10^-6,Ix,L,psi_integral);
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
