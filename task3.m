clear;clc;
qs=[0 0 0];
qf=[2 3 4];
dt=0.01;
v0=0;
vmax=1;%rad/s
accmax=10;
n=0;
while (floor(dt*10^n)~=dt*10^n)
    n=n+1;
end
E = 1*10^-n;
ta= vmax/accmax;
if rem(ta,dt)~=0
    ta_new = round(ta,n)+E;
else
    ta_new = round(ta,n);
end
t1f = (qf(1)-qs(1))/vmax + ta_new;
t2f = (qf(2)-qs(2))/vmax + ta_new;
t3f = (qf(3)-qs(3))/vmax + ta_new;
if rem(t1f,dt)~=0
    t1f_new = round(t1f,n)+E;
else
    t1f_new = round(t1f,n);
end
if rem(t2f,dt)~=0
    t2f_new = round(t2f,n)+E;
else
    t2f_new = round(t2f,n);
end
if rem(t3f,dt)~=0
    t3f_new = round(t3f,n)+E;
else
    t3f_new = round(t3f,n);
end
tf=[t1f_new t2f_new t3f_new];
tf_new=max(tf);

v1_new = ((qf(1)-qs(1))/(tf_new-ta_new));
a1_new = v1_new/ta_new;
v2_new = ((qf(2)-qs(2))/(tf_new-ta_new));
a2_new = v2_new/ta_new;
v3_new = ((qf(3)-qs(3))/(tf_new-ta_new));
a3_new = v3_new/ta_new;

% joint 1 - coefficients:
% t0 --> ta:
a10 = qs(1);
a11 = v0;
a12 = 0.5*a1_new;


% ta --> tf-ta:
a20 = qs(1) + 0.5*a1_new*ta_new^2 - v1_new*ta_new;
a21 = v1_new;

% tf-ta --> tf:
a30 = qf(1) - 0.5*a1_new*tf_new^2;
a31 = a1_new*tf_new;
a32 = -0.5*a1_new;

% joint 2 - coefficients:
% t0 --> ta:
b10 = qs(2);
b11 = v0;
b12 = 0.5*a2_new;


% ta --> tf-ta:
b20 = qs(2) + 0.5*a2_new*ta_new^2 - v2_new*ta_new;
b21 = v2_new;

% tf-ta --> tf:
b30 = qf(2) - 0.5*a2_new*tf_new^2;
b31 = a2_new*tf_new;
b32 = -0.5*a2_new;

% joint 3 - coefficients:
% t0 --> ta:
c10 = qs(3);
c11 = v0;
c12 = 0.5*a3_new;


% ta --> tf-ta:
c20 = qs(3) + 0.5*a3_new*ta_new^2 - v3_new*ta_new;
c21 = v3_new;

% tf-ta --> tf:
c30 = qf(3) - 0.5*a3_new*tf_new^2;
c31 = a3_new*tf_new;
c32 = -0.5*a3_new;

t = 0:dt:tf_new;
q1 = (a10+a11.*t+a12.*t.^2).*(t<=ta_new)...
    +(a20+a21.*t).*(t>ta_new).*(t<=(tf_new-ta_new))...
    +(a30+a31.*t+a32.*t.^2).*(t>(tf_new-ta_new)).*(t<=tf_new);
v1 = (a11+2*a12.*t).*(t<=ta_new)...
    +(a21).*(t>ta_new).*(t<=(tf_new-ta_new))...
    +(a31+2*a32.*t).*(t>(tf_new-ta_new)).*(t<=tf_new);
acc1 = (2*a12).*(t<=ta_new)...
    +(0).*(t>ta_new).*(t<=(tf_new-ta_new))...
    +(2*a32).*(t>(tf_new-ta_new)).*(t<=tf_new);
q2 = (b10+b11.*t+b12.*t.^2).*(t<=ta_new)...
    +(b20+b21.*t).*(t>ta_new).*(t<=(tf_new-ta_new))...
    +(b30+b31.*t+b32.*t.^2).*(t>(tf_new-ta_new)).*(t<=tf_new);
v2 = (b11+2*b12.*t).*(t<=ta_new)...
    +(b21).*(t>ta_new).*(t<=(tf_new-ta_new))...
    +(b31+2*b32.*t).*(t>(tf_new-ta_new)).*(t<=tf_new);
acc2 = (2*b12).*(t<=ta_new)...
    +(0).*(t>ta_new).*(t<=(tf_new-ta_new))....
    +(2*b32).*(t>(tf_new-ta_new)).*(t<=tf_new);
q3 = (c10+c11.*t+c12.*t.^2).*(t<=ta_new)...
    +(c20+c21.*t).*(t>ta_new).*(t<=(tf_new-ta_new))...
    +(c30+c31.*t+c32.*t.^2).*(t>(tf_new-ta_new)).*(t<=tf_new);
v3 = (c11+2*c12.*t).*(t<=ta_new)...
    +(c21).*(t>ta_new).*(t<=(tf_new-ta_new))...
    +(c31+2*c32.*t).*(t>(tf_new-ta_new)).*(t<=tf_new);
acc3 = (2*c12).*(t<=ta_new)...
    +(0).*(t>ta_new).*(t<=(tf_new-ta_new))....
    +(2*c32).*(t>(tf_new-ta_new)).*(t<=tf_new);
 subplot(3,3,1)
 plot(t,q1,'g-')
 title('joint1 position vs time')
 grid on
 subplot(3,3,2)
 plot(t,v1,'b-')
 title('joint1 velocity vs time')
 grid on
  subplot(3,3,3)
 plot(t,acc1,'k-')
 title('joint1 acceleration vs time')
 grid on
  subplot(3,3,4)
 plot(t,q2,'g-')
 title('joint2 position vs time')
 grid on
 subplot(3,3,5)
 plot(t,v2,'b-')
 title('joint2 velocity vs time')
 grid on
  subplot(3,3,6)
 plot(t,acc2,'k-')
 title('joint2 acceleration vs time')
 grid on
  subplot(3,3,7)
 plot(t,q3,'g-')
 title('joint3 position vs time')
 grid on
 subplot(3,3,8)
 plot(t,v3,'b-')
 title('joint3 velocity vs time')
 grid on
  subplot(3,3,9)
 plot(t,acc3,'k-')
 title('joint3 acceleration vs time')
 grid on



