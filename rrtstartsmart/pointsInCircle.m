function q= pointsInCircle(qold,R)
%the function, must be on a folder in matlab path
x1=qold(1);
y1=qold(2);
t=2*pi*rand;
r=sqrt(rand);
x=(R*r)*cos(t)+x1;
y=(R*r)*sin(t)+y1;
q(1,1)=x;
q(1,2)=y;
end