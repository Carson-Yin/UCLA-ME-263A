function T = FK_Demo(c,joint)
% c - array of link lengths
% joint - list of joint variables
% T - T{j} = T0j

d2 = c(1);
a3 = c(2);
a4 = c(3);

d1 = joint(1);
t2 = joint(2);
t3 = joint(3);
t4 = joint(4);

% DH parameters
DH = [0 0 d1 0;pi/2 0 d2 t2;-pi/2 0 0 t3;0 a3 0 t4;0 a4 0 0];
alpha = DH(:,1); a = DH(:,2); d = DH(:,3); theta = DH(:,4);

% initial
To = eye(4); % base frame itself

for j = 1:5
    
    Ti = [cos(theta(j)) -sin(theta(j)) 0 a(j);
        sin(theta(j))*cos(alpha(j)) cos(theta(j))*cos(alpha(j)) ...
        -sin(alpha(j)) -sin(alpha(j))*d(j);
        sin(theta(j))*sin(alpha(j)) cos(theta(j))*sin(alpha(j)) ...
        cos(alpha(j)) cos(alpha(j))*d(j);0 0 0 1];
    To = To*Ti;
    T{j} = To;
    
end

end