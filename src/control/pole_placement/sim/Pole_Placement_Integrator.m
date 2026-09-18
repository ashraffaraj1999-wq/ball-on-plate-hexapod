clear all;
clc;
g=9.81;
K=-5*g/7;
s=tf('s');
X=K/s^2;
Y=K/s^2;
Con=0;
x0=0.3;
y0=0.3;
A=[0 1;0 0];
B=[0;K];
C=[1 0];
Cont=ctrb(A,B);
OBS=obsv(A,C);
Cr=rank(Cont);
Or=rank(OBS);
xi=1.1;
wn=6;
p=[-xi*wn+wn*sqrt(xi^2-1),-xi*wn-wn*sqrt(xi^2-1),-5];
K1=place([A zeros(2,1);-C 0],[B;0],p);
L=(A^2+100*A+2500*eye(2))*OBS^(-1)*[0;1];

%% Plotting Response

plot(time,Xs);
plot(time,Ys);



%% Plotting trajectory

plot(Xs,Ys)
hold on
t=0:0.0001:2*pi;
x=0.3*cos(t);
y=0.3*sin(t);
plot(x,y)
axis equal
hold off