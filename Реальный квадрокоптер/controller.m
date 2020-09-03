 function [W1,W2,W3,W4] = controller(psi,theta,gamma,psi_need,dpsi,m,g,k,b,Ix,L,psi_integral)
 Kd =0;
 Kp =0;
 Ku=0;
 %вычисление ошибки
% e = -Kp*(theta_need-theta)+Kd*(thetadot);%при использовании ПД регулятора
e = Kp*(psi_need-psi)+Kd*(dpsi)+Ku*psi_integral;%при использовании ПИД регулятора
 %вычисление величины сигнала
 W1=128.5-2*b*e*Ix/(4*b*k*L);
 W2= 128.5-(-2*b*e*Ix)/(4*b*k*L);
 W3= 128.5;
 W4= 128.5;
% %  W1= m*g/(4*k*cosd(theta)*cosd(gamma))-2*b*e*Ix/(4*b*k*L);
% %  W2= m*g/(4*k*cosd(theta)*cosd(gamma))-(-2*b*e*Ix)/(4*b*k*L);
% %  W3= m*g/(4*k*cosd(theta)*cosd(gamma));
% %  W4= m*g/(4*k*cosd(theta)*cosd(gamma));
 end
