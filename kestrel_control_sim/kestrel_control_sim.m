clc;
close all;

%drone properties
L = 0.75;

%Runge-Kutta DE solver for system dynamics
f = @(t, x) dynamics(t, x);
tspan = 0: 0.01: 10;
x0 = [pi/2; 0.5; -0.6; 0; 0; 0];
[t, x] = ode45(f, tspan, x0);
figure(1);
plot(t, x(:, 1:3));
legend("yaw", "pitch", "roll");

%animate orientation in time
figure(2);
hold on;
axis_len = 0.25;
h = quiver3(0, 0, 0, axis_len, 0, 0, 'LineWidth',2);
j = quiver3(0, 0, 0, 0, axis_len, 0, 'LineWidth',2);
k = quiver3(0, 0, 0, 0, 0, axis_len, 'LineWidth',2);

f1 = quiver3(0, L, 0, 0, 0, 1, 'LineWidth',2);
f2 = quiver3(0, -L, 0, 0, 0, 1, 'LineWidth',2);

xlim([-2 2]);
ylim([-2 2]);
zlim([-2 2]);
view([-45, 10]);
grid on;
for i = 1:5:1000
 
    R = rot_matrix(x(i,:));

    h_vec = axis_len * R(:,1);
    j_vec = axis_len * R(:,2);
    k_vec = axis_len * R(:,3);

    f1_pos = L * R(:, 2);
    f1_vec = R * [0;0;1];
    
    f2_pos = -L * R(:, 2);
    f2_vec = R * [0;0;1];
    
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
function x_dot = dynamics(t, x)
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
    %moment of inertia matrix
    J = [1 0 0;
         0 1 0;
         0 0 1];

    zero = zeros(3,3);

    A = [zero B; zero zero];
    F = [zero zero; zero J];
    
    %input torques with PD controller
    ref_yaw = 0;
    ref_pitch = 0;
    ref_roll = 0;

    u_yaw = 40*(ref_yaw - x(1)) + 10*(-x_dot_prev(1));
    u_pitch = 40*(ref_pitch - x(2)) + 10*(-x_dot_prev(2));
    u_roll = 40*(ref_roll - x(3)) + 10*(-x_dot_prev(3));

    u = [0; 0; 0; u_roll; u_pitch; u_yaw];
    
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