function points=FK_POS(q)
q1 = q(1,:);
q2 = q(2,:);
q3 = q(3,:);
for i = 1:length(q1)
    x(i) = cos(q1(i))*(cos(q2(i) + q3(i)) + cos(q2(i)));
    y(i) = sin(q1(i))*(cos(q2(i) + q3(i)) + cos(q2(i)));
    z(i) = 1 - sin(q2(i)) - sin(q2(i) + q3(i));
end
points = [x;y;z];
end
