%% plots 
close all;
t_vec1 = 0:dt:t_f;
t_vec2 = 0:dt:t_f_opt;
% trajectory
% figure
draw_cone(p0(1),rad2deg(y_gs));hold on;
plot3(X2(2,:),X2(3,:),X2(1,:),'-k',LineWidth=2); grid on; axis equal; title 'Trajectory'; xlabel x; ylabel y; zlabel z;
hold off;
% position states vs time
figure
plot(t_vec2(1:end-1),X2(1:3,:),LineWidth=1.5); grid on; title 'X,Y,Z vs time';

% throttle lvl
figure
throttle_lvl = [];
for j = 1:size(U2,2)
    throttle_lvl = [throttle_lvl; norm(U2(:,j))*exp(Z2(:,j))/T_max]; %
    % thrust_mag_dvdM = norm(U2(:,j))
    % m = exp(Z2(:,j))
end
plot(t_vec2(1:end-2),throttle_lvl(1:end-1),LineWidth=2); grid on; title 'Throttle lvl';
hold on; yline(.2, '--m', '20%',LineWidth=1); yline(.8, '--r', '80%',LineWidth=1); ylim([0 1]);hold off;
% V_inf against time
figure
V_inf = [];
for j = 1:size(X2,2)
    V_inf = [V_inf; norm(X2(4:6,j))];
end
plot(t_vec2(1:end-1),V_inf,LineWidth=2); grid on; title 'V_{inf}';
hold on; yline(90, '--r','90 m/s',LineWidth=1); ylim([0 100]);hold off;

% thrust direction
figure
thrust_dir = [];
for j = 1:size(X2,2)
    cos_thrust_dir = dot(exp(Z2(:,j))*U2(:,j),nh)/(norm(exp(Z2(:,j))*U2(:,j))*norm(nh));
    thrust_dir = [thrust_dir; acosd(cos_thrust_dir)];
end
plot(t_vec2(1:end-1),thrust_dir(1:end),LineWidth=2); grid on; title 'Thrust Pointing'; 
% ylim([0 50]);
hold on; yline(45, '--r', '45 deg',LineWidth=1); yline(90, '--r', '90 deg',LineWidth=1);hold off;

%%
function draw_cone(alt0,y_gs)
height = alt0;      
angleDegrees = 90 - y_gs; 
nPoints = 50;      

angleRadians = deg2rad(angleDegrees);
radius = height * tan(angleRadians);

theta = linspace(0, 2*pi, nPoints);
r = linspace(0, radius, nPoints);
[Theta, R] = meshgrid(theta, r);

X = R .* cos(Theta);
Y = R .* sin(Theta);
Z = alt0-height * (1 - R/radius); 

% Plot the cone using surf
figure;
h = surf(X, Y, Z);
axis equal; 
title('Upside-Down Cone Visualization');
xlabel('X');
ylabel('Y');
zlabel('Z');

h.FaceColor = [0.7 0.7 0.7]; 
h.EdgeColor = 'none';        
h.FaceAlpha = 0.5;           
end