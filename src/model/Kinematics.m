%% Constants
B=[0 1;0 0];
T=[4 0;0 10];
syms p1 p2 p3 p4
P=[p1 p2;p3 p4];
S=solve(P*B+B'*P == -T,P);
rank(ctrb([0 1;0 0],[0;1]))