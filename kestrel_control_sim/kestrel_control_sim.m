clc;
close all;

%drone properties
L = 0.75;

%Runge-Kutta DE solver for system dynamics
% output state vector: 
% x = [yaw,pitch,roll, w1_dot, w2_dot, w3_dot,T1, T2, theta1, theta2]
f = @(t, x) dynamics(t, x, L);
tspan = 0: 0.01: 10;
x0 = [0.7; 0.25; 0.3; 0; 0; 0];
[t, x] = ode45(f, tspan, x0);
figure(1);
plot(t, x(:,1:3));
legend("yaw", "pitch", "roll");

%extract control inputs from dynamic model
clear dynamics
theta1 = zeros(1,numel(t));
theta2 = zeros(1,numel(t));
T1 = zeros(1,numel(t));
T2 = zeros(1,numel(t));
for i = 1 : numel(t)
    [~, output] = dynamics(t(i)', x(i, :)', L);
    T1(i) = output(1);
    T2(i) = output(2);
    theta1(i) = output(3);
    theta2(i) = output(4);
end

%plot control effort
figure(2)
plot(t, [T1; T2; theta1; theta2]);
legend("thrust1", "thrust2", "theta1", "theta2");

%animate orientation in time
figure(3);
hold on;
axis_len = 0.25;
h = quiver3(0, 0, -0.5, axis_len, 0, 0, 'LineWidth',2);
j = quiver3(0, 0, -0.5, 0, -axis_len, 0, 'LineWidth',2);
k = quiver3(0, 0, -0.5, 0, 0, -axis_len, 'LineWidth',2);

f1 = quiver3(0, L, 0, 0, 0, 0.5, 'LineWidth',2);
f2 = quiver3(0, -L, 0, 0, 0, 0.5, 'LineWidth',2);

xlim([-1 1]);
ylim([-1 1]);
zlim([-1 1]);
view([-45, 10]);
grid on;

for i = 1:5:1000
    %calculate rotation matrix for current orientation
    R = rot_matrix(x(i,:));
    
    %rotate each axis and thrust vectors
    h_vec = axis_len * R(:,1);
    j_vec = -axis_len * R(:,2);
    k_vec = -axis_len * R(:,3);

    f1_pos = L * R(:, 2);
    f1_vec = R * [sin(theta1(i));0;cos(theta1(i))]*0.5;
    
    f2_pos = -L * R(:, 2);
    f2_vec = R * [sin(theta2(i));0;cos(theta2(i))]*0.5;
    
    %update rotation of axes
    set(h, 'UData', h_vec(1), 'VData', h_vec(2), 'WData', h_vec(3));
    set(j, 'UData', j_vec(1), 'VData', j_vec(2), 'WData', j_vec(3));
    set(k, 'UData', k_vec(1), 'VData', k_vec(2), 'WData', k_vec(3));
    
    %update position of thrust vectors
    set(f1, 'XData', f1_pos(1), 'YData', f1_pos(2), 'ZData', f1_pos(3));
    set(f1, 'UData', f1_vec(1), 'VData', f1_vec(2), 'WData', f1_vec(3));
    set(f2, 'XData', f2_pos(1), 'YData', f2_pos(2), 'ZData', f2_pos(3));
    set(f2, 'UData', f2_vec(1), 'VData', f2_vec(2), 'WData', f2_vec(3));

    if i == 1
        pause(1);
    end
    pause(0.05);
end

%rotational dynamics described w/ yaw-pitch-roll euler angles
%integrated with ode45
function [x_dot, output] = dynamics(t, x, L)
    persistent x_dot_prev;
    if isempty(x_dot_prev)
        x_dot_prev = [0 0 0 0 0 0];
    end 

    t
    %y-p-r rotation matrix for angular velocities
    s2 = sin(x(2));
    c2 = cos(x(2));
    s3 = sin(x(3));
    c3 = cos(x(3));
    B = (1/c2)*[0 s3 c3;
                0 c2*c3 -c2*s3;
                c2 s2*s3 s2*c3];
    %thingy
    J = [2 0 0;
         0 2 0;
         0 0 2];

    zero = zeros(3,3);

    A = [zero B; zero zero];
    F = [zero zero; zero J];
    
    %reference yaw pitch and roll angles
    ref_yaw = 0;
    ref_pitch = 0;
    ref_roll = 0;
    
    % control effort yaw pitch and roll (3 PD controllers)
    u_yaw = 2*(ref_yaw - x(1)) + 0.7*(-x_dot_prev(1));
    u_pitch = 2*(ref_pitch - x(2)) + 0.7*(-x_dot_prev(2));
    u_roll = 2*(ref_roll - x(3)) + 1*(-x_dot_prev(3));
    
    %control mixing algorithm
    T1 = 1.7 + u_roll;
    T2 = 1.7 - u_roll;
    theta1 = u_yaw - u_pitch;
    theta2 = -u_yaw - u_pitch;
    
    %thrust vector angle limiting
    if abs(theta1) > pi/8
        theta1 = sign(theta1) * pi/8;
    end

    if abs(theta2) > pi/8
        theta2 = sign(theta2) * pi/8;
    end
    
    %extract additional values
    output = [T1 T2 theta1 theta2];
    
    %calculate moments caused by thrust vectors
    m1 = (T1*cos(theta1) - T2*cos(theta2)) * L;
    m2 = (T1*sin(-theta1) + T2*sin(-theta2)) * L;
    m3 = (T1*sin(theta1) + T2*sin(-theta2)) * L;

    u = [0; 0; 0; m1; m2; m3];
    
    %Differential Equation
    x_dot = A*x + F*u;
    x_dot_prev = x_dot;
end

%y-p-r rotation matrix given euler angles
function R = rot_matrix(theta)
    c1 = cos(theta(1));
    c2 = cos(theta(2));
    c3 = cos(theta(3));
    s1 = sin(theta(1));
    s2 = sin(theta(2));
    s3 = sin(theta(3));

    R = [c2*c1 c2*s1 -s2;
         s3*s2*c1-c3*s1 s3*s2*s1+c3*c1 s3*c2;
         c3*s2*c1+s3*s1 c3*s2*s1-s3*c1 c3*c2];
end