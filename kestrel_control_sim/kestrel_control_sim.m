clc;
close all;

%Runge-Kutta DE solver for system dynamics
f = @(t, x) dynamics(t, x);
tspan = [0: 0.01: 100];
%x0 = [40*pi/180; 30*pi/180; 80*pi/180; 0; 0; 0];
x0 = [1.1; 0; 0; 0; 0; 0];
[t, x] = ode45(f, tspan, x0);
figure(1);
plot(t, x);
legend("yaw", "pitch", "roll", "alpha1", "alpha2", "alpha3");

%animate orientation in time
figure(2);
hold on;
h = quiver3(0, 0, 0, 1, 0, 0, 'LineWidth',2);
j = quiver3(0, 0, 0, 0, 1, 0, 'LineWidth',2);
k = quiver3(0, 0, 0, 0, 0, 1, 'LineWidth',2);
xlim([-1 1]);
ylim([-1 1]);
zlim([-1 1]);
view([-37.5, 30]);
grid on;

for i = 1:90:10000
 
    R = rot_matrix(x(i,:));
 
    h.UData = R(1, 1);
    h.VData = R(2, 1);
    h.WData = R(3, 1);

    j.UData = R(1, 2);
    j.VData = R(2, 2);
    j.WData = R(3, 2);

    k.UData = R(1, 3);
    k.VData = R(2, 3);
    k.WData = R(3, 3);

    if i == 1
        pause(1);
    end
    pause(0.05);
end

%rotational dynamics described w/ yaw-pitch-roll euler angles
function x_dot = dynamics(t, x)
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
    
    %torque input
    u = [0; 0; 0; 0.1*(- x(3))+0.4*(-x(4)); 0.1*(- x(2))+0.4*(-x(5)); 0.1*(- x(1))+0.4*(-x(6))];
    
    %Differential Equation
    x_dot = A*x + F*u;
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