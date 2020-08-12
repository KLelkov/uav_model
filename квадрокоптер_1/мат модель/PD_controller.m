 function W = controller(theta_need,theta,thetadot_need,thetadot,m,g,k,b,I,L,Kd,Kp)
Kd =1.8;
 Kp =1.5;
 %вычисление ошибки
 e = -Kp*(theta_need-theta)+Kd*(thetadot);
 %вычисление величины сигнала
 W(1)=m*g/(4*k*cosd(theta(2))*cosd(theta(1)))-(2*b*e(1)*I(1,1)+e(3)*I(3,3)*k*L)/(4*b*k*L);
 W(2)=m*g/(4*k*cosd(theta(2))*cosd(theta(1)))+ e(3)*I(3,3)/(4*b)-e(2)*I(2,2)/(2*k*L);
 W(3)=m*g/(4*k*cosd(theta(2))*cosd(theta(1)))- (-2*b*e(1)*I(1,1)+e(3)*I(3,3)*k*L)/(4*b*k*L);
 W(4)=m*g/(4*k*cosd(theta(2))*cosd(theta(1)))+e(3)*I(3,3)/(4*b)+e(2)*I(2,2)/(2*k*L);
 end