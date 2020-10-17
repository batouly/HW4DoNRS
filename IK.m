function q=IK(X)
x=X(1);
y=X(2);
z=X(3);
L1=1;L2=1;L3=1;
q1 = atan2(y, x);
r=sqrt(x^2+y^2);
s=L1-z;
d=sqrt(r^2+s^2);
D=(d^2-L2^2-L3^2)/(2*L2*L3);
q3=atan2(sqrt(1-D^2),D);
B=L3*sqrt(1-D^2)/d;
q2=atan2(s,r)-atan2(B,sqrt(1-B^2));
q=[q1 q2 q3];
end