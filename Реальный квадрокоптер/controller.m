  function [W1,W2,W3,W4] = controller(theta_need,theta,thetadot,m,g,k,b,I,L,theta_integral, params)
%  Kp = params(1);
%  Kd = params(2);
%  Ku = params(3);
 Kd =1.9;
 Kp =-3.2;
 Ku=-0.1;
 %вычисление ошибки
% e = -Kp*(theta_need-theta)+Kd*(thetadot);%??? ????????????? ?? ??????????
e = -Kp*(theta_need-theta)+Kd*(thetadot)+Ku*theta_integral;%??? ????????????? ??? ??????????
 %вычисление величины сигнала
 W1=128.5-(2*b*e(1)*I(1)+e(3)*I(3)*k*L)/(4*b*k*L);
 W2=128.5 + e(3)*I(3)/(4*b)-e(2)*I(2)/(2*k*L);
 W3=128.5- (-2*b*e(1)*I(1)+e(3)*I(3)*k*L)/(4*b*k*L);
 W4=128.5+e(3)*I(3)/(4*b)+e(2)*I(2)/(2*k*L);
 end
