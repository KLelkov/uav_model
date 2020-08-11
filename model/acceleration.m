function a = acceleration(inputs, angles, xdot, m, g, k, cd,S,r0)
 gravity = [0; 0; -g];
R = rotation(angles);
 T = R * thrust(inputs, k);
 Fd =-sign(xdot)*cd*r0*xdot'.^2*S/2;
 a = gravity + 1 / m * T + diag(Fd)/m;
 end