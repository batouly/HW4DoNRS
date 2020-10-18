clear;clc;
p1=[1 0 1];
p2=[0.7071 0.7071 1.2];
dt=0.01;
vmax=1; %m/s
amax=10;
N = 100;
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
tf=max(tf);
tspan = linspace(0,tf,N);
% positions
x = ((p2(1)-p1(1))/tf).*tspan+p1(1);
y = ((p2(2)-p1(2))/tf).*tspan+p1(2);
z = ((p2(3)-p1(3))/tf).*tspan+p1(3);
v1 = (p2(1)-p1(1))/tf;
v2= (p2(2)-p1(2))/tf;
v3 = (p2(3)-p1(3))/tf;
vel = [v1;v2;v3];
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
     A = [1 t1 t1^2 t1^3
        0 1 2*t1 3*t1^2
        1 t2 t2^2 t2^3
        0 1 2*t2 3*t2^2];
    for j = 1:3
         qs = jointConfig(i,j); qf = jointConfig(i+1,j);
         v0 = jointVel(i,j); vf = jointVel(i+1,j);
          c = [qs;v0;qf;vf];
         b = A\c;
         t = linspace(t1,t2,n);
         q_n(j,:) = b(1)+b(2).*t+b(3).*t.^2+b(4).*t.^3;
         qd_n(j,:) = b(2)+2*b(3).*t+3*b(4).*t.^2;
    end   
    Q = [Q q_n];
    Qd = [Qd qd_n];
end 
cartTraj = FK_POS(Q);
CartV=Jac(Q)*Qd;
ns = n*N-n;
figure
plot(tspan, x )
hold on 
T = linspace(0,tf,ns);
plot(T,cartTraj(1,:),'k--')
grid on
legend('X_{des}','x_{actual}')
figure
plot(tspan, y )
hold on 
T = linspace(0,tf,ns);
plot(T,cartTraj(2,:),'k--')
grid on
legend('y_{des}','y_{actual}')
figure
plot(tspan, z )
hold on 
T = linspace(0,tf,ns);
plot(T,cartTraj(3,:),'k--')
grid on
legend('z_{des}','z_{actual}')
figure
subplot(311)
plot(T, Q(1,:))
ylabel('joint 1 position')
grid on
subplot(312)
plot(T, Q(2,:))
ylabel('joint 2 position')
grid on
subplot(313)
plot(T, Q(3,:))
ylabel('joint 3 position')
grid on
figure
subplot(311)
plot(T, Qd(1,:))
ylabel('joint 1 velocity')
grid on
subplot(312)
plot(T, Qd(2,:))
ylabel('joint 2 velocity')
grid on
subplot(313)
plot(T, Qd(3,:))
ylabel('joint 3 velocity')
grid on



