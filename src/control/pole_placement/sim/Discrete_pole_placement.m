g=9.81;
Ts=0.02;
Con=1;
x0=0.1;
y0=0.15;
up=6*pi/180;
down=-up;
K=5*g/7;
s=tf('s');
X=6.433/s^2;
Y=K/s^2;
A_c=[0 1;0 0];
B_c=[0;K];
C_c=[1 0];
D_c=0;
Cont=ctrb(A_c,B_c);
OBS=obsv(A_c,C_c);
Cr=rank(Cont);
Or=rank(OBS);
sys=ss(A_c,B_c,C_c,D_c);
sys_d=c2d(sys,Ts);
[A_d,B_d,C_d,D_d,~]=ssdata(sys_d);
xi=1.1;
wn=2;
p=[-xi*wn+wn*sqrt(xi^2-1),-xi*wn-wn*sqrt(xi^2-1)];
p_d=exp(Ts*p);
K1=place(A_d,B_d,p_d);
sys_d=ss(A_d-B_d*K1,B_d,C_d,D_d,Ts);
%step(sys_d);
steady=D_d+C_d*(eye(2)-(A_d-B_d*K1))^(-1)*B_d;
%%bode(sys_d)

Con_Obs_poles=[-15 -15];
Dis_Obs_poles=exp(Con_Obs_poles*Ts);
sum=Dis_Obs_poles(1)+Dis_Obs_poles(2);
prod=Dis_Obs_poles(1)*Dis_Obs_poles(2);
OBS_d=obsv(A_d,C_d);
L=(A_d^2+sum*A_d+prod*eye(2))*(OBS_d)^(-1)*[0;1];

%% Plotting Response

plot(time,Xs);
plot(time,Ys);

%% Plotting trajectory

plot(Xs,Ys)
hold on
t=0:0.0001:2*pi;
axis(equal)
x=0.3*cos(t);
y=0.3*sin(t);
plot(x,y)
hold off