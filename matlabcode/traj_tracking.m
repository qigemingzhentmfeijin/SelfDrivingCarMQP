 clear; clc;
% Initial and final condition (x, y, theta)
q0 = [0, -4, 0];
qf = [10, 2, 0];
Tf = 10;				% time to reach destination

syms t
a = sym('a', [1,4]); % the parameters of trajectory for x
b = sym('b', [1,4]); % the parameters of trajectory for y


basis = [1; t; t^2; t^3];

dbasis = [0; 1; 2*t; 3*t^2];
xsym = a*basis;

ysym = b*basis;
dx = a*dbasis;

dy = b*dbasis;


x0 = subs(xsym,t,0);

xf = subs(xsym,t,Tf);

dx0 = subs(dx,t,0);

dxf = subs(dx,t,Tf);

y0 = subs(ysym,t,0);
yf = subs(ysym,t,Tf);
dy0 = subs(dy,t,0);
dyf = subs(dy,t,Tf);

% compute the jacobian linearization of the vector field.
l=1;
syms v w theta x y
f= [v*cos(theta); v*sin(theta); w];

dfdx = jacobian(f,  [x;y;theta]);
dfdu = jacobian(f,  [v;w]);

% solve linear equations for finding the coefficients in the feasible
% trajectories.

% % initial and terminal condition: with velocity equals zero.
% [matA,matb] = equationsToMatrix([x0==q0(1), y0==q0(2), dx0*sin(q0(3))+  dy0*cos(q0(3))==1, dx0*cos(q0(3))+ dy0*sin(q0(3))==1, ...
% xf==qf(1), yf==qf(2), dxf*sin(qf(3))+ dyf*cos(qf(3))==1, dxf*cos(qf(3))+ dyf*sin(qf(3))==1],[a(1),a(2),a(3),a(4),b(1),b(2),b(3),b(4)]);
% initial and terminal condition: with velocity equals zero.
[matA,matb] = equationsToMatrix([x0==q0(1), y0==q0(2), dx0*sin(q0(3))-  dy0*cos(q0(3))==0, dx0*sin(q0(3))+ dy0*cos(q0(3))==0, ...
xf==qf(1), yf==qf(2), dxf*sin(qf(3))- dyf*cos(qf(3))==0, dxf*sin(qf(3))+ dyf*cos(qf(3))==0],[a(1),a(2),a(3),a(4),b(1),b(2),b(3),b(4)]);
param = matA\matb;

[matA,matb] = equationsToMatrix([x0==q0(1), y0==q0(2), dx0*sin(q0(3))-  dy0*cos(q0(3))==0, dx0*sin(q0(3))+ dy0*cos(q0(3))==0, ...
xf==qf(1), yf==qf(2), dxf*sin(qf(3))- dyf*cos(qf(3))==0, dxf*sin(qf(3))+ dyf*cos(qf(3))==0],[a(1),a(2),a(3),a(4),b(1),b(2),b(3),b(4)]);
param = matA\matb;

avec= double(param(1:4)');
bvec = double(param(5:end)');


% now apply the feedback controller
ode_tracking;
