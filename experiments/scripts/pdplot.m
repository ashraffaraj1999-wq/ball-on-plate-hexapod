
first=752;
last=1674;
ref=[ones(1,first)*15,...
    ones(1,last-first)*20,ones(1,length(x1)-last)*10];
t=(1:length(x1))*0.02;
plot(t,x1,t,y1,t,ref)