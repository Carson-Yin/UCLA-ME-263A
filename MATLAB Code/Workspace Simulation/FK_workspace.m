function T = FK_workspace(c,joint)
% c - constant parameters
% joint - joint positions list
% T - T{i} = T0i

a3 = c(1);
a4 = c(2);
d2 = c(3);
d1 = joint(1);
t2 = joint(2);
t3 = joint(3);
t4 = joint(4);

% Input DH Parameters
% Depend on specific problems
alpha = [0 (pi/2) -(pi/2) 0 0];
a = [0 0 0 a3 a4];
d = [d1 d2 0 0 0];
th = [0 t2 t3 t4 0];

% initial
T_ground = [cos(-pi/2) sin(pi/2)*sin(-pi/2) cos(pi/2)*sin(-pi/2) 0;
            0 cos(pi/2) -sin(pi/2) 0;
            -sin(-pi/2) cos(-pi/2)*sin(pi/2) cos(pi/2)*cos(-pi/2) 0;
            0 0 0 1];
temp = T_ground; % base frame itself


    
for i = 1:length(a)
    Ti = TF(a(i),alpha(i),th(i),d(i));
    temp = temp * Ti;
    T{i} = temp;  
end

function T_temp = TF(a,alpha,theta,d)
    T_temp = [cos(theta) -sin(theta) 0 a;
        sin(theta)*cos(alpha) cos(theta)*cos(alpha) -sin(alpha) -sin(alpha)*d;
        sin(theta)*sin(alpha) cos(theta)*sin(alpha) cos(alpha) cos(alpha)*d;
        0 0 0 1;];
end

end