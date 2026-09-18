%% Lagrangian Definition

syms rb mb Ib Ip xb yb alpha beta g
syms dxb dyb dalpha dbeta ddxb ddyb ddalpha ddbeta
syms Tb(mb,rb,dxb,dyb,Ib)
syms Tp(Ip,Ib,dalpha,dbeta,xb,yb)
syms V(mb,g,xb,alpha,yb,beta)
syms L(mb,Ip,Ib,rb,dalpha,dbeta,dxb,dyb)
syms q dq
q=[xb,yb,alpha,beta,dxb,dyb,dalpha,dbeta];
dq=[dxb;dyb;dalpha;dbeta;ddxb;ddyb;ddalpha;ddbeta];
Tb(mb,rb,dxb,dyb,Ib)=0.5*(mb+(Ib/rb)^2)*(dxb^2+dyb^2);
Tp(Ip,Ib,dalpha,dbeta,xb,yb)=0.5*(Ib+Ip)*(dalpha^2+dbeta^2)+0.5*mb*(xb*dalpha+yb*dbeta)^2;
V(mb,g,xb,alpha,yb,beta)=mb*g*(xb*sin(alpha)+yb*sin(beta));
L(mb,Ip,Ib,rb,dalpha,dbeta,dxb,dyb)=Tb(mb,rb,dxb,dyb,Ib)+Tp(Ip,Ib,dalpha,dbeta,xb,yb)- ... 
    V(mb,g,xb,alpha,yb,beta);

eq1=matlabFunction(jacobian(diff(L,dxb),q)*dq)-diff(L,xb);
eq2=matlabFunction(jacobian(diff(L,dyb),q)*dq)-diff(L,yb);
eq3=matlabFunction(jacobian(diff(L,dalpha),q)*dq)-diff(L,alpha);
eq4=matlabFunction(jacobian(diff(L,dbeta),q)*dq)-diff(L,beta);
eq=simplify([eq1;eq2;eq3;eq4])

%% Linearized Model (state space)
