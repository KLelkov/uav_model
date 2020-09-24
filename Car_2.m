clear
clc
alfa(1) = 0;
gamma(1) =0;
CTE(1) =0;
V = 1;
L =0.1;%расстояние между колесами
l = 50*0.1;%длина машины/2
x0(1) =0;
y0(1)=0;
dt=0.1;
theta(1) =0;
d= 50*L/2;
sigma = atand(d/l);
x1(1) = x0+d*cosd(sigma); y1(1) = y0 + d*sind(sigma);
x2(1) = x0+d*cosd(sigma); y2(1) = y0-d*sind(sigma);
x3(1) = x0-d*cosd(sigma); y3(1) = y0-d*sind(sigma);
x4(1) = x0-d*cosd(sigma); y4(1) = y0+d*sind(sigma);
% массив траекторных точек
x_need = [0 25 50 25 -25 -50 -25 0];
y_need = [0 25 0 -25 25 0 -25 0];
j =1;
delta = 1.5;
%для ПИД регулятора
% K=42;
Kp_V =0.00;
Kd_V =0.000;
% Kp_alfa =K;
% Kp_gamma =K;
% Kd_alfa =-0.9;
% Kd_gamma =-0.8;
     K=97.1;
     Kd = 830;
     Kp_alfa =K;
Kp_gamma =K;
Kd_alfa =Kd;
Kd_gamma =Kd;
for i =1:400/dt
    if j > numel(x_need) - 1
        break;
    end
    
    delta_x = x_need(j+1)-x_need(j); delta_y = y_need(j+1)-y_need(j);
    Vx = (V(i)*cosd(alfa(i))+ V(i)*cosd(90+gamma(i)))*dt;
    Vy = (V(i)*cosd(270+alfa(i))+V(i)*cosd(gamma(i)))*dt;
dtheta =((V(i)*sind(alfa(i)) - V(i)*sind(gamma(i)))/L)*dt;
theta(i+1) = theta(i)+dtheta;
x0(i+1) = x0(i)+Vx*cosd(theta(i+1));
y0(i+1) = y0(i)+Vy*sind(theta(i+1));
 x1(i+1) = x0(i+1)+d*cosd(sigma+theta(i+1)); y1(i+1) = y0(i+1) + d*sind(sigma+theta(i+1));
 x2(i+1) = x0(i+1)+d*cosd(sigma+theta(i+1)); y2(i+1) = y0(i+1)- d*sind(sigma+theta(i+1));
 x3(i+1) = x0(i+1)-d*cosd(sigma+theta(i+1)); y3(i+1) = y0(i+1)- d*sind(sigma+theta(i+1));
 x4(i+1) = x0(i+1)-d*cosd(sigma+theta(i+1)); y4(i+1) = y0(i+1)+ d*sind(sigma+theta(i+1));

 %Для ПИД регулятора
 Rx = x0(i+1)-x_need(j); Ry = y0(i+1)-y_need(j);
  %
 U = (Rx*delta_x + Ry*delta_y) / (delta_x^2 + delta_y^2);
 %
 CTE(i+1) = (Ry*delta_x-Rx*delta_y)/(delta_x^2+delta_y^2);
 alfa(i+1) = -(Kp_alfa*CTE(i+1) + Kd_alfa*((CTE(i+1)-CTE(i))/dt));
 gamma(i+1) = Kp_gamma*CTE(i+1)+ Kd_gamma*((CTE(i+1)-CTE(i))/dt);
 V(i+1) = V(i) + Kp_V*CTE(i+1)+ Kd_V*((CTE(i+1)-CTE(i))/dt);
%  if abs(x0(i+1)-x_need(j+1))<delta && (abs(y0(i+1)-y_need(j+1))<delta)
 if U > 1
     j= j+1;
 end
 
end
figure
plot(x0,y0)
grid on
hold on
plot(x_need, y_need, '--k')
