clear all;
clc;
Ts=0.02;
up=5*pi/180;
down=-up;
g=9.81;
K=5*g/7;
s=tf('s');
X=K/s^2;
Y=K/s^2;
xi=0.7;
wn=1;
Kd=2*xi*wn/K;
Kp=wn^2/K;
step(K*(Kp+Kd*s)/(s^2+K*Kd*s+K*Kp))
Ts=0.02;
z=tf('z',Ts);
C=Kp+(Kd/Ts)*(1-z^(-1));
%% Plotting Response

Xs=out.Xs;
Ys=out.Ys;
T=out.time;
plot(T,Xs);
plot(T,Ys);



%% Plotting trajectory

Xs=out.Xs;
Ys=out.Ys;
plot(Xs,Ys)
hold on
t=0:0.0001:2*pi;
x=cos(t);
y=sin(t);
plot(x,y)
hold off