g=9.81;
Ts=0.02;
x0=0.3;
y0=0.3;
K=5*g/7;
A_c=[0 1 0 0;0 0 0 0;0 0 0 1;0 0 0 0];
B_c=[0 0;K 0;0 0;0 K];
C_c=[1 0 0 0;0 0 1 0]; 
D_c=[0 0;0 0];
sys=ss(A_c,B_c,C_c,D_c);
sys_d=c2d(sys,Ts);
[A_d,B_d,C_d,D_d,~]=ssdata(sys_d);
A=A_d;
B=B_d;
C=C_d;
D=D_d;
nx=4;nu=2;ny=2;N=50;Nu=5;
sum=zeros(nx,nx);
Mx=[];
V=[];
for i=1:N
    sum=sum+A^(i-1);
    V=[V;sum];
end
for i=1:N
    temp=[];
    for j=1:Nu
        if(j > i)
            temp=[temp zeros(nx,nu)];
        else
            temp=[temp V(i-j+1)*B];
        end
    end
    Mx=[Mx;temp];
end
A_bar=[];
for i=1:N
    A_bar=[A_bar;A^i];
end
C_bar=[];
for i=1:N
    temp=[];
    for j=1:N
    if(j == i)
        temp=[temp C];
    else
        temp=[temp zeros(ny,nx)];
    end
    end
    C_bar=[C_bar;temp];
end
I_bar=[];
for i=1:N
    I_bar=[I_bar eye(ny)];
end
Psi=eye(N*ny);
Lambda=eye(Nu*nu);
K=[eye(nu) zeros(nu,nu*Nu-nu)]*...
    (((Mx')*(C_bar')*Psi*C_bar*Mx+Lambda)^(-1))*(Mx')*(C_bar')*Psi;
Ke=zeros(nu,ny);
G1=zeros(nu,nx);
G2=zeros(nu,nx);
for i=1:N
    Ke=Ke+K(:,i:i+1);
    G1=G1+K(:,i:i+1)*C*V(i);
    G2=G2+K(:,i:i+1)*C*A^i;
    i=i+2;
end
