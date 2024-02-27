%get data from simulink model
theta_euler = out.euler_ang.Data;
theta_rotor = out.rotor_angle.Data;

L = 0.75;

%animate orientation in time
anim = figure(1);
set(anim,'Position', [250 100 900 600]);
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

for i = 1:5:300
    %calculate rotation matrix for current orientation
    R = rot_matrix(theta_euler(i,:));
    
    %rotate each axis
    h_vec = axis_len * R(:,1);
    j_vec = -axis_len * R(:,2);
    k_vec = -axis_len * R(:,3);

    %rotate thrust vectors
    f1_pos = L * R(:, 2);
    f1_vec = R * [sin(theta_rotor(i, 1));0;cos(theta_rotor(i, 1))]*0.5;
    
    f2_pos = -L * R(:, 2);
    f2_vec = R * [sin(theta_rotor(i, 2));0;cos(theta_rotor(i, 2))]*0.5;
    
    %update rotation of axes
    set(h, 'UData', h_vec(1), 'VData', h_vec(2), 'WData', h_vec(3));
    set(j, 'UData', j_vec(1), 'VData', j_vec(2), 'WData', j_vec(3));
    set(k, 'UData', k_vec(1), 'VData', k_vec(2), 'WData', k_vec(3));
    
    %update position of thrust vectors
    set(f1, 'XData', f1_pos(1), 'YData', f1_pos(2), 'ZData', f1_pos(3));
    set(f1, 'UData', f1_vec(1), 'VData', f1_vec(2), 'WData', f1_vec(3));
    set(f2, 'XData', f2_pos(1), 'YData', f2_pos(2), 'ZData', f2_pos(3));
    set(f2, 'UData', f2_vec(1), 'VData', f2_vec(2), 'WData', f2_vec(3));
    
    %short delay to allow plot to show up
    if i == 1
        pause(1);
    end
    pause(0.05);
end

%r-p-y rotation matrix given euler angles
function R = rot_matrix(theta)
    c1 = cos(theta(1));
    c2 = cos(theta(2));
    c3 = cos(theta(3));
    s1 = sin(theta(1));
    s2 = sin(theta(2));
    s3 = sin(theta(3));

    R = [c3*c2 c3*s2*s1+s3*c1 -c3*s2*c1+s3*s1
        -s3*c2 -s3*s2*s1+c3*c1 s3*s2*c1+c3*s1
         s2 -c2*s1 c2*c1];
end