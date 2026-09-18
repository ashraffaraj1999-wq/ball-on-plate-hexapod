%%
up=4.5*pi/180;
down=-up;
g=9.81;
K1=6.1486;
K2=6.664;
Ts=0.02;
A_x=[0 1;0 0];
B_x=[0;K1];
C_x=[1 0];
D_x=0;
D_y=0;
A_y=[0 1;0 0];
B_y=[0;K2];
C_y=[1 0];
tr=3;
xi=1.1;
wx=6.9/tr;
wy=6.9/tr;
p_x=[-xi*wx+wx*sqrt(xi^2-1),-xi*wx-wx*sqrt(xi^2-1)];
p_y=[-xi*wy+wy*sqrt(xi^2-1),-xi*wy-wy*sqrt(xi^2-1)];
p_d_x=exp(Ts*p_x);
p_d_y=exp(Ts*p_y);

sys_x=ss(A_x,B_x,C_x,D_x);
sys_d_x=c2d(sys_x,Ts);
[A_d_x,B_d_x,C_d_x,D_d_x,~]=ssdata(sys_d_x);
Kx=place(A_d_x,1.1*B_d_x,p_d_x);
Lx=place(A_d_x',C_d_x',p_d_x*0.2);

sys_y=ss(A_y,B_y,C_y,D_y);
sys_d_y=c2d(sys_y,Ts);
[A_d_y,B_d_y,C_d_y,D_d_y,~]=ssdata(sys_d_y);
Ky=place(A_d_y,1.1*B_d_y,p_d_y);
Ly=place(A_d_y',C_d_y',p_d_y*0.2);