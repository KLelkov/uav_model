 function W = controller(theta_need,theta,thetadot_need,thetadot,m,g,k,b,I,L,theta_integral, params)
%  Kp = params(1);
%  Kd = params(2);
%  Ku = params(3);
 Kd =1.9;
 Kp =3.2;
 Ku=-0.1;
 %���������� ������
% e = -Kp*(theta_need-theta)+Kd*(thetadot);%��� ������������� �� ����������
e = -Kp*(theta_need-theta)+Kd*(thetadot)+Ku*theta_integral;%��� ������������� ��� ����������
 %���������� �������� �������
 W(1)=m*g/(4*k*cosd(theta(2))*cosd(theta(1)))-(2*b*e(1)*I(1,1)+e(3)*I(3,3)*k*L)/(4*b*k*L);
 W(2)=m*g/(4*k*cosd(theta(2))*cosd(theta(1)))+ e(3)*I(3,3)/(4*b)-e(2)*I(2,2)/(2*k*L);
 W(3)=m*g/(4*k*cosd(theta(2))*cosd(theta(1)))- (-2*b*e(1)*I(1,1)+e(3)*I(3,3)*k*L)/(4*b*k*L);
 W(4)=m*g/(4*k*cosd(theta(2))*cosd(theta(1)))+e(3)*I(3,3)/(4*b)+e(2)*I(2,2)/(2*k*L);
 end
% %  ������������ ����� ������� �� 250 ��������:
% %  Kp = 4.12024, Kd = 2.764, Ku = 0.237104