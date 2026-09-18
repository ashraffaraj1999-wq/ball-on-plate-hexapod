function [ th ] = inv_ken(px,py,pz,alpha,beta,gamma)
% Constants
l1=3.2;
l2=25.57;
b=26;
d=20;
b1=30;
d1=6;
RA1=[(1/sqrt(12))*(2*b+d);d/2;0];
RA2=[-(1/sqrt(12))*(b-d);(b+d)/2;0];
RA3=[-(1/sqrt(12))*(b+2*d);b/2;0];
RA4=[-(1/sqrt(12))*(b+2*d);-b/2;0];
RA5=[-(1/sqrt(12))*(b-d);-(b+d)/2;0];
RA6=[(1/sqrt(12))*(2*b+d);-d/2;0];
RA=[RA1 RA2 RA3 RA4 RA5 RA6];

RC1=[(1/sqrt(12))*(b1+2*d1);b1/2;0];
RC2=[(1/sqrt(12))*(b1-d1);(b1+d1)/2;0];
RC3=[-(1/sqrt(12))*(2*b1+d1);d1/2;0];
RC4=[-(1/sqrt(12))*(2*b1+d1);-d1/2;0];
RC5=[(1/sqrt(12))*(b1-d1);-(b1+d1)/2;0];
RC6=[(1/sqrt(12))*(b1+2*d1);-b1/2;0];
RC=[RC1 RC2 RC3 RC4 RC5 RC6];

v=[0 0 0 1];
T_base_A1=[rotz(150)*rotx(90) RA1;v];
T_base_A2=[rotz(-30)*rotx(90) RA2;v];
T_base_A3=[rotz(-90)*rotx(90) RA3;v];
T_base_A4=[rotz(90)*rotx(90) RA4;v];
T_base_A5=[rotz(30)*rotx(90) RA5;v];
T_base_A6=[rotz(-150)*rotx(90) RA6;v];

% Computation
R=[rotz(gamma)*roty(beta)*rotx(alpha);0 0 0];
P=[px;py;pz;1];
T=[R P];
RCE=T*[RC;ones(1,6)];
RAE=[RA;ones(1,6)];
L=zeros(1,6);
for i =1:6
    L(i)=sum((RCE(:,i)-RAE(:,i)).^2)^0.5;
end

RA1C1=T_base_A1^(-1)*(RCE(:,1)-RAE(:,1));
RA2C2=T_base_A2^(-1)*(RCE(:,2)-RAE(:,2));
RA3C3=T_base_A3^(-1)*(RCE(:,3)-RAE(:,3));
RA4C4=T_base_A4^(-1)*(RCE(:,4)-RAE(:,4));
RA5C5=T_base_A5^(-1)*(RCE(:,5)-RAE(:,5));
RA6C6=T_base_A6^(-1)*(RCE(:,6)-RAE(:,6));
RAC=[RA1C1 RA2C2 RA3C3 RA4C4 RA5C5 RA6C6];
P1=zeros(1,6);
G1=zeros(1,6);
for i=1:6
    P1(i)=(L(i)^2+l1^2-l2^2)/(2*l1);
end
for i=1:6
    G1(i)=RAC(1,i)^2+RAC(2,i)^2-(P1(i)^2);
end
delta=1;
th=zeros(1,6);
for i=1:6
    th(i)=atan2d(P1(i)*RAC(2,i)+RAC(1,i)*delta*sqrt(G1(i)), ...
        abs(P1(i)*RAC(1,i)-RAC(2,i)*delta*sqrt(G1(i))));
end
end

