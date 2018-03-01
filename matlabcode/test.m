clc;clear;
%syms x0 xb0 x1 x2;
syms xd0 xd1 xd2;
% syms x y theta;
syms x_e y_e theta_e;
% syms pi1 pi2;
syms u1 u2;
syms a b c dummy1;
syms x_r y_r theta_r w_r v_r theta x y;
m = 1;
k1 = 2

[gamma0, gamma1, gamma2] = deal(0.5);


dummy1 = [ cos(theta) sin(theta) 0;...
          -sin(theta) cos(theta) 0;...
           0          0          1 ]*...
        [x_r-x;...
         y_r-y;...
         theta_r-theta];
[x0, x1, x2] = deal(dummy1(3), dummy1(2), -dummy1(1));

pi1 = (x1^2 + x2^2 + 1)^0.5;
xb0 = m*x0 + x1/pi1;
pi2 = (xb0^2 + 1)^0.5;
x2
xb0
f1 = [ (x2/pi1) * w_r + ((1+x2^2)/pi1^3) * v_r * sin(xb0/m - x1/(m*pi1));...
        x2 * w_r + v_r * sin(xb0/m - x1/(m*pi1));...
       -w_r * x1 ];

g1 = [  m - x2/pi1  -(x1*x2)/(pi1^3);...
       -x2           0;...
        x1           1];
    
%V = sqrt(xb0^2 + 1) + k1 * sqrt(x1^2 + x2^2 + 1) - (1 + k1);

dVdx = [ xb0/pi2 k1*(x1/pi1) k1*(x2/pi1)];
V = dVdx*(f1+g1*[u1;u2]);

%V = @(u) dVdx*(f1+g1*[u(1);u(2)]);

diff(V,u1)
diff(V,u2)
%dxf = dVdx*f1;





(((cos(theta)*(x - x_r) + sin(theta)*(y - y_r))/((cos(theta)*(x - x_r) + sin(theta)*(y - y_r))^2 + (cos(theta)*(y - y_r) - sin(theta)*(x - x_r))^2 + 1)^(1/2) - 1)*(theta - theta_r + (cos(theta)*(y - y_r) - sin(theta)*(x - x_r))/((cos(theta)*(x - x_r) + sin(theta)*(y - y_r))^2 + (cos(theta)*(y - y_r) - sin(theta)*(x - x_r))^2 + 1)^(1/2)))/((theta - theta_r + (cos(theta)*(y - y_r) - sin(theta)*(x - x_r))/((cos(theta)*(x - x_r) + sin(theta)*(y - y_r))^2 + (cos(theta)*(y - y_r) - sin(theta)*(x - x_r))^2 + 1)^(1/2))^2 + 1)^(1/2)


