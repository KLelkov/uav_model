  function [W1,W2,W3,W4] = controller(theta_need,theta,thetadot,m,g,k,b,I,L,theta_integral, Kp,Kd,Ku)
%  Kp =3.86489;
%  Kd =3.1282;
%  Ku=2.47186;
Kp = 3.5;
Kd =1.2;
Ku=3.3;
L=0.36/2;
 %вычисление ошибки
% e = -Kp*(theta_need-theta)+Kd*(thetadot);
e = -Kp*(theta_need-theta)+Kd*(thetadot)+Ku*theta_integral;
 %вычисление величины сигнала
 W1=128.5 + L*I(3)*e(1)/4/k+e(2)*L*I(1)/k/4+e(3)*L*I(2)/b/4;
 W2=128.5 - L*I(3)*e(1)/4/k + e(2)*L*I(1)/k/4 - e(3)*L*I(2)/b/4;
 W3=128.5 -  L*I(3)*e(1)/4/k - e(2)*L*I(1)/k/4 + e(3)*L*I(2)/b/4;
 W4=128.5 + L*I(3)*e(1)/4/k - e(2)*L*I(1)/k/4 - e(3)*L*I(2)/b/4;
 end
