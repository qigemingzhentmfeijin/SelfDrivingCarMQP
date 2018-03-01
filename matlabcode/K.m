clc;
%K Summary of this function goes here
%   Detailed explanation goes here
syms alpha beta theta;
w_max = 0.5;
w_min = 0.5;
v_max = 20;
e_v = 2.0;
e_w = 0.2;
v_min = 1;
gamma_0 = 10;
m = 1;
k1 = 2;
gamma1 = 0.5;
gamma2 = 0.5;


rho1 = (k1*beta+alpha/...
    ((alpha^2+1)^0.5)*theta)*sin((alpha-beta)/m)+k1*gamma1*beta*sin(beta/m);
rho2 = alpha/((alpha^2+1)^0.5)*(w_max-e_w)+(k1-0.5)*(v_min*(v_min+e_v)*cos((alpha-beta)/m))+gamma2*(k1-0.5)*((v_min+e_v)*cos(beta/m)-v_min);

M3 = (alpha^2+1)*rho1/(alpha^2);
diff(M3, alpha)
alpha = 1;



M3 = 0;
M4 = 0;
M5 = 0;


d3 = M3*(v_max-e_v)+gamma_0 + 0.5*(w_max-e_w);
d4 = M3*(v_max-e_v)+gamma_0 + max(M4,M5);
kw = max(2*w_max*e_w,d3/(m-1),d4/(m-1));


