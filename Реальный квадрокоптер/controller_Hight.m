 function [W1,W2,W3,W4] = controller_Hight(y,y_need,Vy)
 Kp = 10;
 Kd =13;
 e = Kp*(y-y_need)+Kd*Vy;
 W1 = 128.5 -e;
 W2 = 128.5 -e;
 W3 = 128.5 -e;
 W4 = 128.5 -e;
 end