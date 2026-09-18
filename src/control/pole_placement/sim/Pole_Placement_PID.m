clear all;
clc;
up=5*pi/180;
down=-up;
g=9.81;
K2=6.664;
Ts=0.02;
K1=6.1486;
s=tf('s');
X=K1/s^2;
Y=K2/s^2;
Con=0;
x0=0.3;
y0=0.3;
A_x=[0 1;0 0];
B_x=[0;K1];
C_x=[1 0];
D_x=0;
D_y=0;
A_y=[0 1;0 0];
B_y=[0;K2];
C_y=[1 0];
Cont_x=ctrb(A_x,B_x);
Obs_x=obsv(A_x,C_x);
Cont_y=ctrb(A_y,B_y);
Obs_y=obsv(A_y,C_y);
set_time=3;
xi=0.7;
wn=1/0.7;
Hdes = wn^2/(s^2+2*xi*wn*s+wn^2);
pol = pole(Hdes);
p=[-xi*wn+wn*sqrt(xi^2-1),-xi*wn-wn*sqrt(xi^2-1)];
Kx=place(A_x,1.2*B_x,p);
Ky=place(A_y,1.2*B_y,p);

sys_res_x=ss(A_x-B_x*Kx,B_x*Kx,C_x,D_x);
sys_res_y=ss(A_y-B_y*Ky,B_y*Ky,C_y,D_y);
step(sys_res_x)
step(sys_res_y)
L_x=(A_x^2+50*A_x+225*eye(2))*Obs_x^(-1)*[0;1];

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
hold off