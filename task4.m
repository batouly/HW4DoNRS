clear;clc;
p1=[1 0 1];
p2=[0.7071 0.7071 1.2];
dt=0.01;
vmax=1; %m/s
amax=10;
tf = 10;
N = 100;
tspan = linspace(0,tf,N);
% positions
x = ((p2(1)-p1(1))/tf).*tspan+p1(1);
y = ((p2(2)-p1(2))/tf).*tspan+p1(2);
z = ((p2(3)-p1(3))/tf).*tspan+p1(3);
n=0;
while (floor(dt*10^n)~=dt*10^n)
    n=n+1;
end
E = 1*10^-n;
ta= vmax/amax;
if rem(ta,dt)~=0
    ta_new = round(ta,n)+E;
else
    ta_new = round(ta,n);
end
t1f = (p2(1)-p1(1))/vmax + ta_new;
t2f = (p2(2)-p1(2))/vmax + ta_new;
t3f = (p2(3)-p1(3))/vmax + ta_new;
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
v1_new = ((p2(1)-p1(1))/(tf_new-ta_new));
a1_new = v1_new/ta_new;
v2_new = ((p2(2)-p1(2))/(tf_new-ta_new));
a2_new = v2_new/ta_new;
v3_new = ((p2(3)-p1(3))/(tf_new-ta_new));
a3_new = v3_new/ta_new;
vel = [v1_new;v2_new;v3_new];
jointConfig=zeros(N,3);
jointVel=zeros(N,3);
for i=1:N
    waypnts = [x(i), y(i), z(i)];
    jointConfig(i,:) = IK(waypnts);
    if i == 1 || i==N
        jointVel(i,:) = [0 0 0];
    else
        J = Jac(jointConfig(i,:));
        jointVel(i,:) =  (J\vel)';
    end
end
% for each joitn:
n = 50;
Q = [];
Qd = [];
for i=1:N-1
    t1 = tspan(i); t2 = tspan(i+1);
    for j = 1:3
         qs = jointConfig(i,j); qf = jointConfig(i+1,j);
         v0 = jointVel(i,j); vf = jointVel(i+1,j);
         t = linspace(t1,t2,n);
         
         % joint - coefficients:
            % t0 --> ta:
            a10 = qs;
            a11 = v0;
            a12 = 0.5*a1_new;
            % ta --> tf-ta:
            a20 = qs + 0.5*a1_new*ta_new^2 - v1_new*ta_new;
            a21 = v1_new;
            % tf-ta --> tf:
            a30 = qf - 0.5*a1_new*t2^2;
            a31 = a1_new*t2;
            a32 = -0.5*a1_new;
            q_n(j,:) = (a10+a11.*t+a12.*t.^2).*(t<=ta_new)...
                +(a20+a21.*t).*(t>ta_new).*(t<=(t2-ta_new))...
                +(a30+a31.*t+a32.*t.^2).*(t>(t2-ta_new)).*(t<=t2);
            qd_n(j,:) = (a11+2*a12.*t).*(t<=ta_new)...
                +(a21).*(t>ta_new).*(t<=(t2-ta_new))...
                +(a31+2*a32.*t).*(t>(t2-ta_new)).*(t<=t2);
    end   
    Q = [Q q_n];
    Qd = [Qd qd_n];
end 
cartTraj = FK_POS(Q);




