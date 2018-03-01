function [ x ] = CLFF( x_r,y_r,theta_r,w_r,v_r, x, y, theta, old)
%CLFF Summary of this function goes here
%   Detailed explanation goes here


%syms x0 xb0 x1 x2;
syms xd0 xd1 xd2;
% syms x y theta;
syms x_e y_e theta_e;
% syms pi1 pi2;
syms u1 u2;
syms a b c dummy1;

cc1 = 0.5;
cc2 = 1.2;


m = 1;
k1 = 2;
e_v = 1.5;
v_min = 8;
v_max = 13;
ng0 = 10;
ng1 = 10;

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

%c1 = double(diff(V,u1));
c1 = (((cos(theta)*(x - x_r) + sin(theta)*(y - y_r))/((cos(theta)*(x - x_r) + sin(theta)*(y - y_r))^2 + (cos(theta)*(y - y_r) - sin(theta)*(x - x_r))^2 + 1)^(1/2) - 1)*(theta - theta_r + (cos(theta)*(y - y_r) - sin(theta)*(x - x_r))/((cos(theta)*(x - x_r) + sin(theta)*(y - y_r))^2 + (cos(theta)*(y - y_r) - sin(theta)*(x - x_r))^2 + 1)^(1/2)))/((theta - theta_r + (cos(theta)*(y - y_r) - sin(theta)*(x - x_r))/((cos(theta)*(x - x_r) + sin(theta)*(y - y_r))^2 + (cos(theta)*(y - y_r) - sin(theta)*(x - x_r))^2 + 1)^(1/2))^2 + 1)^(1/2);

%c2 = double(diff(V,u2));
c2 = (2*(cos(theta)*(x - x_r) + sin(theta)*(y - y_r)))/((cos(theta)*(x - x_r) + sin(theta)*(y - y_r))^2 + (cos(theta)*(y - y_r) - sin(theta)*(x - x_r))^2 + 1)^(1/2) - ((cos(theta)*(x - x_r) + sin(theta)*(y - y_r))*(cos(theta)*(y - y_r) - sin(theta)*(x - x_r))*(theta - theta_r + (cos(theta)*(y - y_r) - sin(theta)*(x - x_r))/((cos(theta)*(x - x_r) + sin(theta)*(y - y_r))^2 + (cos(theta)*(y - y_r) - sin(theta)*(x - x_r))^2 + 1)^(1/2)))/(((theta - theta_r + (cos(theta)*(y - y_r) - sin(theta)*(x - x_r))/((cos(theta)*(x - x_r) + sin(theta)*(y - y_r))^2 + (cos(theta)*(y - y_r) - sin(theta)*(x - x_r))^2 + 1)^(1/2))^2 + 1)^(1/2)*((cos(theta)*(x - x_r) + sin(theta)*(y - y_r))^2 + (cos(theta)*(y - y_r) - sin(theta)*(x - x_r))^2 + 1)^(3/2));
%dxf = dVdx*f1;
F = [-c1 c2];

%V = symfun(vpa(dVdx*(f1+g1*[u0;u1])), [u0;u1]);
%V = symfun(0.01940285000290663788151251696929 - 0.43547331708194531648665083960825*u1 - 0.84768475726092403496694842051068*u0, [u0 u1]);

%W = gamma0*(xb0/pi2)^2 + gamma1*k1*(v_min+e_v)*(x1/pi1)*sin(x1/(m*pi1)) + ...
%    gamma2*(k1-1/2)*(x2/pi1)^2*((v_min + e_v)*cos(x1/(m*pi1))-v_min);

%thetaa = old(2)/0.4*tan(0.5)
%tan(0.5)
H = [1 0;0 1;-1 0;0 -1];
%bound by speed
g = [0.5-old(2)/18*0.4, min(20, old(2)+cc2) 0.5-old(2)/18*0.4 min([-1.2,-old(2)+cc2,-0.1])];
%SteerUB = [1.0,w_r+0.15,0.5-old(2)/18*0.4]
%SteerLB = [1.0,-(w_r-0.15),0.5-old(2)/18*0.4]
SteerUB = [3/old(2), tan(0.5)*old(2)/0.4,w_r+0.5];
SteerLB = [3/old(2), tan(0.5)*old(2)/0.4,-(w_r-0.5)];

%bound by traj
g = [min(SteerUB), min(20, old(2)+cc2), min(SteerLB), min([-1.2,-old(2)+2*cc2,-0.1])];

%g = [min(0.5, old(1)+cc1) min(15, old(2)+cc2) min(0.5, -old(1)+cc1) min([-1.1,-old(2)+cc2,-0.1])];
nw = 2;
nv = 3.5;
u0 = sat(nw*xb0,-1.0,1.0);
u1 = sat(-nv*x2,0.1,20);
x = [u0,u1];
%   -1<u1<1   -3<u2<15

% options = optimoptions('linprog','Algorithm','interior-point');
% 
% %[x,fvel] = fmincon(V,[-3,0],H,f,[],[],[-1,0],[1,100]); x =
% %linprog(F,H,g,[],[],[],[],options);
% x = linprog(F,H,g);
% 




end

