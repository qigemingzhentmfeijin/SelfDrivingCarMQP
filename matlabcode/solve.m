syms x0 x1 x2 xb0;
syms theta x y thetar xr yr;
syms pi1 pi2;

m = 1;
k1 = 2;
xe = [cos(theta) sin(theta) 0;-sin(theta) cos(theta) 0; 0 0 1]*[xr-x;yr-y;thetar-theta];
x0 = xe(3);
x1 = xe(2);
x2 = xe(1);

pi1 = (x1^2+x2^2+1)^0.5;
xb0 = m*x0+x1/pi1;
pi2 = (xb0^2)^0.5;

V = (xb0^2+1)^0.5+k1*(x1^2+x2^2+1)^0.5+-(1+k1);


"===================================="


