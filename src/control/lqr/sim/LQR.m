clear all; 
clc;
g=9.81;
Ts=0.02;
K1=6.1486;
K2=6.664;
up=5*pi/180;
down=-up;

A_x=[0 1;0 0];
B_x=[0;K1];
C_x=[1 0];
D_x=0;
D_y=0;
A_y=[0 1;0 0];
B_y=[0;K2];
C_y=[1 0];

Q=diag([5;5]);
R=10;

sys_x=ss(A_x,B_x,C_x,D_x);
sys_d_x=c2d(sys_x,Ts);
[A_d_x,B_d_x,C_d_x,D_d_x,~]=ssdata(sys_d_x);
Kx=dlqr(A_d_x,B_d_x,Q,R);

sys_y=ss(A_y,B_y,C_y,D_y);
sys_d_y=c2d(sys_y,Ts);
[A_d_y,B_d_y,C_d_y,D_d_y,~]=ssdata(sys_d_y);
Ky=dlqr(A_d_y,B_d_y,Q,R);






%% Observer

Con_Obs_poles=[-10 -10];
Dis_Obs_poles=exp(Con_Obs_poles*Ts);
sum=Dis_Obs_poles(1)+Dis_Obs_poles(2);
prod=Dis_Obs_poles(1)*Dis_Obs_poles(2);
OBS_d=obsv(A_d,C_d);
L=(A_d^2+sum*A_d+prod*eye(2))*(OBS_d)^(-1)*[0;1];

%% Plotting Response

plot(time,Xs);
figure()
plot(time,Ys);

%% Plotting Trajectory

plot(Xs,Ys)
hold on
t=0:0.0001:2*pi;
x=0.3*cos(t);
y=0.3*sin(t);
plot(x,y)
axis equal
hold off
