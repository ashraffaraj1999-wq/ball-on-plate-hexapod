%%
clear
clc
up=5*pi/180;
down=-up;
g=9.81;
Ts=0.01;
K=-5*g/7;
xim=1.1;
wm=2;
Am=[-2*xim*wm -wm^2;1 0];
Bm=[1;0];
Cm=[0 wm^2];
Dm=0;
A=[0 1;0 0];
B=[0;K];
C=[1 0];
D=0;
Q=10*eye(2);
gammax=diag([35 10]);
gammar=1;
syms p11 p12 p21 p22
P=[p11 p12;p21 p22];
S=solve(P*Am+Am'*P==-Q,P);
P_bar=[double(S.p12);double(S.p22)];
%% Plot circle
t=0:0.0001:2*pi;
x=0.15*cos(t);y=0.15*sin(t);
plot(x,y)
hold on
plot(Xs(3000:6001),Ys(3000:6001))