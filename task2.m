q10 = 0; q1f = 2; v10 = 0; v1f = 0;acc10 = 0;acc1f = 0;
q20 = 0; q2f = 3;  v20 = 0; v2f = 0; acc20 = 0;  acc2f = 0;
q30 = 0; q3f = 4;v30=0; v3f=0; acc30=0; acc3f=0;
t0=0;tf=2;
A = [1 t0 t0^2 t0^3 t0^4 t0^5
     0 1 2*t0 3*t0^2 4*t0^3 5*t0^4
     0 0 2 6*t0 12*t0^2 20*t0^3
     1 tf tf^2 tf^3 tf^4 tf^5
     0 1 2*tf 3*tf^2 4*tf^3 5*tf^4
     0 0 2 6*tf 12*tf^2 20*tf^3];
c1 = [q10;v10;acc10;q1f;v1f;acc1f];
b1 = A\c1;
% computing for joint 1
a01 = b1(1); a11 = b1(2); a21 = b1(3); a31 = b1(4); a41 = b1(5); a51 = b1(6);
t = t0:0.01:tf;
 q1 = a01+a11.*t+a21.*t.^2+a31.*t.^3+a41.*t.^4+a51.*t.^5;
 v1 = a11+2*a21.*t+3*a31.*t.^2+4*a41.*t.^3+5*a51.*t.^4;
 acc1 = 2*a21+6*a31.*t+12*a41.*t.^2+20*a51.*t.^3;
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
 % computing for joint 2
c1 = [q20;v20;acc20;q2f;v2f;acc2f];
b1 = A\c1;
% computing for joint 1
a01 = b1(1); a11 = b1(2); a21 = b1(3); a31 = b1(4); a41 = b1(5); a51 = b1(6);
t = t0:0.01:tf;
 q2 = a01+a11.*t+a21.*t.^2+a31.*t.^3+a41.*t.^4+a51.*t.^5;
 v2 = a11+2*a21.*t+3*a31.*t.^2+4*a41.*t.^3+5*a51.*t.^4;
 acc2 = 2*a21+6*a31.*t+12*a41.*t.^2+20*a51.*t.^3;
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
 % computing for joint 3
 c1 = [q30;v30;acc30;q3f;v3f;acc3f];
b1 = A\c1;
a01 = b1(1); a11 = b1(2); a21 = b1(3); a31 = b1(4); a41 = b1(5); a51 = b1(6);
t = t0:0.01:tf;
 q3 = a01+a11.*t+a21.*t.^2+a31.*t.^3+a41.*t.^4+a51.*t.^5;
 v3 = a11+2*a21.*t+3*a31.*t.^2+4*a41.*t.^3+5*a51.*t.^4;
 acc3 = 2*a21+6*a31.*t+12*a41.*t.^2+20*a51.*t.^3;
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