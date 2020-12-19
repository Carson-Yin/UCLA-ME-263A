% MAE 263A Project
% Simulation

clear all
clc;
% clf;
clear;

% Link Lengths
d2 = 0.38; % m
a3 = 0.3; % m
a4 = 0.28; % m
c = [d2 a3 a4];

% Trajectory in Cartesian Space
% N = 100;
% t=linspace(-1,1);
% z=0.1*t;
% y=-0.1+0*t;
% x=-0.2+0*t;

% Joint Space
% for each end effector position p and orientation R calculate inverse
% kinematics
% for i = 1:N
%     p = [x(i);y(i);z(i)];
% %     R = [1 0 0;0 1 0;0 0 1];
%     R = [-1 0 0;0 -1 0;0 0 1];
% %     ca = cos(0.5*t(i));
% %     sa = sin(0.5*t(i));
% %     R = [1 0 0;0 ca -sa;0 sa ca];
%     T0e = [R p;0 0 0 1];
%     joint(:,i) = IK(T0e,c); % joint holds the values of all of the joint variables for all N EE points
% end

%FK Joint Space for demo purpose
h=(a3+a4)/8;    % surface height below x=0

t=linspace(-1,1);
ai=15*pi/180;   % angle increment for t4 for -y motion of arm in s trajectory
ang=linspace(0,ai,25);
sa=-15*pi/180;
% joint1a=[0.6*t;0*t;-pi/2+0*t;pi/4+0*t];
joint1a=[0.6*t;0*t;acos((h-a4*cos(sa+0))/a3)+pi+0*t;0+2*pi-acos((h-a4*cos(sa+0))/a3)+sa+0*t];
joint2a=[0.6*1+0*ang;0*ang;acos((h-a4*cos(sa+ang))/a3)+pi;ang+2*pi-acos((h-a4*cos(sa+ang))/a3)+sa];
joint3a=[-0.6*t;0*t;joint2a(3,end)+0*t;joint2a(4,end)+0*t];
joint4a=[-0.6+0*ang;0*ang;acos((h-a4*cos(sa+ai+ang))/a3)+pi;2*pi-acos((h-a4*cos(sa+ai+ang))/a3)+sa+ai+ang];
joint5a=[0.6*t;0*t;joint4a(3,end)+0*t;joint4a(4,end)+0*t];
joint6a=[0.6+0*ang;0*ang;acos((h-a4*cos(sa+2*ai+ang))/a3)+pi;2*pi-acos((h-a4*cos(sa+2*ai+ang))/a3)+sa+2*ai+ang];
joint7a=[-0.6*t;0*t;joint6a(3,end)+0*t;joint6a(4,end)+0*t];

joint_old = [joint1a joint2a joint3a joint4a joint5a joint6a joint7a];  %  joint2a joint3a joint4a joint5a joint6a
[M,N]=size(joint_old);
% N = 175;
speed = 5;

for i = 1:speed:N
    T_old = FK_Demo(c,joint_old(:,i));
    T0e{i} = T_old{5};
    position{i} = T_old{5}(1:3,end);
end
%get trajectory
for i = 1:speed:N
    x{i} = position{i}(1);
    y{i} = position{i}(2);
    z{i} = position{i}(3);
end
%convert to double for plotting
plot_x = cell2mat(x);
plot_y = cell2mat(y);
plot_z = cell2mat(z);
% IK Joint Space
% for each end effector position p and orientation R calculate inverse
% kinematics
for i = 1:speed:N
    joint_test(:,i) = IK_Demo(T0e{i},c); % joint holds the values of all of the joint variables for all N EE points
end

% Plot Setting
movie = 1;
% Create Movie
if movie == 1
    v = VideoWriter('IK_Demo.mp4','MPEG-4');
    open(v);
end

% Plot
figure (1)

for i = 1:speed:N

T = FK_Demo(c,joint_test(:,i));   % T holds transformation matrices from base to each frame

% Trajectory
plot3(plot_x,plot_y,plot_z,'c','linewidth',2);
hold on;

% Manipulator
for j = 1:3
    pj = T{j}(1:3,4);
    pj1 = T{j+1}(1:3,4);
    plot3([pj(1) pj1(1)],[pj(2) pj1(2)],[pj(3) pj1(3)],'k','linewidth',4);
    hold on;
end
% Tool
p4 = T{4}(1:3,4);
p5 = T{5}(1:3,4);
plot3([p4(1) p5(1)],[p4(2) p5(2)],[p4(3) p5(3)],'m','linewidth',4);

% Frames
for j = 1:5
    pj = T{j}(1:3,4);
    Rj = T{j}(1:3,1:3);
    scale = 0.1;
    xj = pj + Rj(:,1)*scale;
    yj = pj + Rj(:,2)*scale;
    zj = pj + Rj(:,3)*scale;
    plot3([pj(1) xj(1)],[pj(2) xj(2)],[pj(3) xj(3)],'r','linewidth',2); % x-axes
    plot3([pj(1) yj(1)],[pj(2) yj(2)],[pj(3) yj(3)],'g','linewidth',2); % y-axes
    plot3([pj(1) zj(1)],[pj(2) zj(2)],[pj(3) zj(3)],'b','linewidth',2); % z-axes
end

% Base
plot3([0 0],[0 0],[-0.8 0.8],'k','linewidth',8);

% Ground
% X = [1 -1;1 -1]*0.2;
% Y = [1 1;-1 -1]*0.2;
% Z = [1 1;1 1]*-0.2;

X = linspace(-0.8,0.8);  
Y = 0*X; 
Z = [zeros(size(X))+0.8; zeros(size(X))-0.8];
surf([X; X], [Y; Y], Z)

% Setting
xlabel('x [m]');
ylabel('y [m]');
zlabel('z [m]');
axis([-0.8 0.8 -1.2 0.8 -0.8 0.8]);
pbaspect([1 1 1]);
grid on;

view([1,-1,1]);
camup([1 0 0]);
cameratoolbar('Show')
cameratoolbar('SetCoordSys','none')
cameratoolbar('SetMode','orbit')

hold off

if movie == 1
    frame = getframe(gcf);
    writeVideo(v,frame);
else
    drawnow
end

end

if movie == 1
    close(v)
end