x = 0;
y = 0;
theta = 0;
xr = 0;
yr = 0;
thetar = 0;

fe = [cos(theta) sin(theta) 0;-sin(theta) cos(theta) 0; 0 0 1]*[xr-x;yr-y;thetar-theta];

x0, x1, x2 = fe(3), fe(2), fe(1);
























% xE = 1;
% yE = 0;
% thetaE = 0;
% thetaDes = 1;
% vDes = 1;
% 
% 
% k1 = 2;
% x0 = thetaE;
% x1 = yE;
% x2 = -1*xE;
% m = 1;
% pi1 = (x1^2+x2^2+1)^0.5;
% 
% 
% 
% F = x2/pi1*thetaDes+(1+x2^2)/pi1*vDes*sin(x0/m-x1/(m*pi1));
% g = [m-x2/pi1 -x1*x2/(pi1^3)];
% V = (x0^2+1)^0.5+k1*(x1^2+x2^2+1)^0.5-(1+k1);
% 
% vv0 = F+V;
% vv1 = g;
% vv0
% vv1