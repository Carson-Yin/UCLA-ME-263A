clc;
clear all

% Parameter
a3 = 0.30; % m
a4 = 0.28; % m
d2 = 0.414; % m
c = [a3 a4 d2];
for d1 = 0:0.1:1
    for t2 = -pi/12:0.01:pi/12 %it can be iin [-pi, pi]
        for t3 = -pi/12:0.1:pi/12 %it can be iin [-pi, pi]
            for t4 = -pi/12:0.1:pi/12 %it can be iin [-pi, pi]
        
joint_test = [d1;t2;t3;t4];%[d1 t2 t3 t4]
T = FK_workspace(c,joint_test);

% Plot base frame relative to MATLAB's base frame
figure (1)
R_ground = [cos(-pi/2) sin(pi/2)*sin(-pi/2) cos(pi/2)*sin(-pi/2);
            0 cos(pi/2) -sin(pi/2);
            -sin(-pi/2) cos(-pi/2)*sin(pi/2) cos(pi/2)*cos(-pi/2)];
base_x = transpose(R_ground*[0.02;0;0]);
base_y = transpose(R_ground*[0;0.02;0]);
base_z = transpose(R_ground*[0;0;0.02]);
plot3([0 base_x(1)],[0 base_x(2)],[0 base_x(3)],'r','linewidth',2); % x-axes
hold on
plot3([0 base_y(1)],[0 base_y(2)],[0 base_y(3)],'g','linewidth',2); % y-axes
hold on
plot3([0 base_z(1)],[0 base_z(2)],[0 base_z(3)],'b','linewidth',2); % z-axes
hold on
set(gca,'zdir','reverse')
set(gca,'ydir','reverse')
xlabel('Y [m]');
ylabel('Z [m]');
zlabel('X [m]');
pbaspect([1 1 1]);
grid on;
view(40,30);
hold on;
p4 = T{5}(1:3,4);
plot3(p4(1),p4(2),p4(3),'.','Color','b','MarkerSize',5,'MarkerFaceColor','#D9FFFF');% plot point representing end-effector
hold on
            end
        end
    end
end

title('Workspace')