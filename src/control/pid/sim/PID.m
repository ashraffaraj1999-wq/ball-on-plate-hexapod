clear all;
clc;
g=9.81;
Ts=0.02;
up=5*pi/180;
down=-up;
K=5*g/7;
s=tf('s');
X=K/s^2;
Y=K/s^2;
syms ki kp kd
K_con=[kd;kp;ki];
tr=3;
xi=1.1;
wn=1;
p=[-xi*wn+wn*sqrt(xi^2-1),-xi*wn-wn*sqrt(xi^2-1)];
g=conv(conv([1 -p(1)],[1 -p(2)]),[1 8]);
S=solve([1;K*K_con] == g' , K_con);
KI=double(S.ki);
Kp=double(S.kp);
Kd=double(S.kd);
C=KI/s+Kp+Kd*s;
z=tf('z',Ts);
C1=Kp+(Kd/Ts)*(1-z^(-1))+KI*Ts*z/(z-1);
Res=C*X/(1+C*X);
[num,den]=tfdata(Res,'v');
num=num(2:4);
den=den(1:4);
Res=tf(num,den);
step(Res)
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