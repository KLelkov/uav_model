% Compute thrust given current inputs and thrust coefficient
function thetadot =  omega2thetadot(omega, theta)
thetadot = [1 0 -sind(theta(2)); 0 cosd(theta(1)) cosd(theta(2))*sind(theta(1)); 0 -sind(theta(1)) cosd(theta(2))*cosd(theta(1))]^-1*omega;
 end