 function [W1,W2,W3,W4] = controller_velocity(Vy,Vy_need)
 Kp = 4;
 e = Kp*(Vy-Vy_need);
 W1 = 128.5 -e;
 W2 = 128.5 -e;
 W3 = 128.5 -e;
 W4 = 128.5 -e;
 end