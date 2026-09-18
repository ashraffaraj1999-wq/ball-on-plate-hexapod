%% root locus
xi=0.5;
wn=2;
p1=-xi*wn+wn*sqrt(1-xi^2)*1i;
num=10;
den=[1 0 0];
poles_org=roots(den);
zeros_org=roots(num);
phi=0;
for j=1:length(poles_org)
    phi=phi+angle(p1-poles_org(j));
end
for j=1:length(zeros_org)
    phi=phi+angle(p1-zeros_org(j));
end
phi=abs(pi-phi);
anglezero=(angle(p1)+phi)/2;
anglepole=(angle(p1)-phi)/2;
xpole=real(p1)-imag(p1)/tan(anglepole);
xzero=real(p1)-imag(p1)/tan(anglezero);
K=abs((p1+xpole)/(p1+xzero))*abs(polyval(den,p1)/polyval(num,p1));
s=tf('s');
Gc=K*(s-xzero)/(s-xpole);
G=10/s^2;
numtot=conv(num,[K,-K*xzero]);
dentot=conv(den,[1,-xpole]);
dentot=dentot+[zeros(1,length(dentot)-length(numtot)) numtot];
Gtot=tf(numtot,dentot,s);
pole(Gtot)
step(Gtot);
%% stablizing sets

