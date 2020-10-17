function J=Jac(q)
T=FK(q);
L1=1;L2=1;L3=1;
 % Geometrical Jacobian
O0 = [0 0 0]';
z0 = [0 0 1]';
O=T(1:3,4);
R=T(1:3,1:3);
T1=Tz(L1)*Rz(q(1));
T2=T1*Ry(q(2))*Tx(L2);
R2=T2(1:3,1:3);
O1=T1(1:3,4);
O2=T2(1:3,4);
R1=T1(1:3,1:3);
z1 = R1*[0 1 0]';
z2 = R2*[0 1 0]';

% The first joint (revolute)
Jv1 = cross(z0, O - O0);
Jw1 = z0;

% The second joint (revolute)
Jv2 = cross(z1, O - O1);
Jw2 = z1;

% The third joint (revolute)
Jv3 = cross(z2, O - O2);
Jw3 = z2;

Jv = [Jv1, Jv2, Jv3];
%Jw = [Jw1, Jw2, Jw3];
J = [Jv];
end