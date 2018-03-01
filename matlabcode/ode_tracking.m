%function dxvec = ode_tracking(t,xvec,avec, bvec)
% evaluate the desired state.
clc;
dt=0.1;
Tf = 10;

tsteps=[0:dt:Tf];

N=size(tsteps,2);

X = zeros(5,N);
uplot = zeros(2,N);
vplot = zeros(1,N);
% with some initial error
% with no-initial error 
X(:,1)=[0, -8, 0.0, 0, 0];

%x,y,theta,w,v


Xdes = zeros(5,N);
u = [0 0];

oldu = u;
for i=1:N-1
    xvec = X(:,i);
i
%x = xvec(1);
%y = xvec(2);
theta = xvec(3);

theta= wrapTo2Pi(theta);

%theta = theta - 2*pi*floor(theta/(2*pi));



%l=1;
t=tsteps(i);
basis = [1; t; t^2; t^3];
dbasis = [0; 1; 2*t; 3*t^2];
ddbasis = [0; 0;2; 6*t];

xdes = avec*basis;
dxdes = avec*dbasis;
ddxdes = avec*ddbasis;

ydes = bvec*basis;
dydes = bvec*dbasis;
ddydes = bvec*ddbasis;
V = (dxdes^2+dydes^2)^0.5;
R = abs(((dxdes^2+dydes^2)^(3/2))/(dxdes*ddydes-dydes*ddxdes));
W = V/R;
% compute sin(theta_d)

thetades = atan2(dydes, dxdes);
Xdes(:,i)= [xdes;ydes;thetades;W;V];

% The desired state.
xdes_vec = [xdes; ydes; thetades;W;V];

% compute the feedforward in the input.
vf = dxdes*cos(thetades) + dydes*sin(thetades);
dthetades = 1/vf*(ddydes*cos(thetades) - ddxdes*sin(thetades));
wf = dthetades;


%end

oldu = u;

u = CLFF(xdes,ydes,thetades,W,V,X(1,i),X(2,i),X(3,i),oldu);
tha = atan(u(1)*0.4/u(2));
u
uplot(i) = tha;
%u is output
%calculate xvec  :
%omega = tan(u(2))*vf;
dxvec = [u(2)*cos(theta);u(2)*sin(theta);u(1);wf;vf];

% 
% % without noise
%calculate xvec  :  
X(:,i+1)= dxvec*dt+ X(:,i);

% with noise
%X(:,i+1)= dxvec*dt+ X(:,i) +0.05*randn(1);
% [pub,msg] = rospublisher('/racer/matlab','std_msgs/Float32');
% 
% 
% %pub = rospublisher('/racer/ACC/nextSpeed','std_msgs/Float32');
% %msg = rosmessage(pub);
% msg.Data = i;
% send(pub,msg);


%X(:,i+1)= X(:,i+1)+0.05*randn(1);

end


% sub = rossubscriber('/racer/ACC/nextSpeed','std_msgs/Float32')
% pause(1);
% msg2 = receive(sub)
% 
% sub.LatestMessage.Data

for i = N:N
    u = CLFF(xdes,ydes,thetades,W,V,X(1,i),X(2,i),X(3,i),oldu);
    oldu = u;
    tha = atan(u(1)*0.4/u(2));
    u
    uplot(i) = tha;
    vplot(i) = u(2);

    dxvec = [u(2)*cos(theta);u(2)*sin(theta);u(1);wf;vf];

    X(:,i+1)= dxvec*dt+ X(:,i);

end

for i=1:N;
    t=tsteps(i);
    basis = [1; t; t^2; t^3];
dbasis = [0; 1; 2*t; 3*t^2];
ddbasis = [0; 0;2; 6*t];
Xdes(1,i) = avec*basis;
    Xdes(2,i)= bvec*basis;
end

figure(1);

plot(X(1,:), X(2,:),'LineWidth', 4);
%plot(uplot(1,:));
hold on 
plot(Xdes(1,:), Xdes(2,:), 'LineWidth', 4);



%hold on 
%plot(Xdes(1,:), Xdes(2,:), 'LineWidth', 4);