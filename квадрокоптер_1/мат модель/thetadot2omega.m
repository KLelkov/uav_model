% Compute thrust given current inputs and thrust coefficient
function omega =  thetadot2omega(thetadot, theta)
omega = [1 0 -sind(theta(2)); 0 cosd(theta(1)) cosd(theta(2))*sind(theta(1)); 0 -sind(theta(1)) cosd(theta(2))*cosd(theta(1))]*thetadot;
 end