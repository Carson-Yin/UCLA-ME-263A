function joint = IK_Demo(T0e,c)
% T0e - end-effector transform matrix
% c - parameter
% joint - joint position list

d2 = c(1);
a3 = c(2);
a4 = c(3);

r = T0e(1:3,1:3);
x = T0e(1,4);
y = T0e(2,4);
z = T0e(3,4);

t2=atan2(-r(1,3),r(3,3));

t3=atan2((y-a4*r(2,1)+d2)*r(3,3),x-a4*r(3,3)*r(2,2));

t4=atan2(r(2,1),r(2,2))-t3;

d1=z+x*r(1,3)/r(3,3);

joint = [d1;t2;t3;t4];

end