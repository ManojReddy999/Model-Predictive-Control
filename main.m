clear all; close all; clc;

% Field Dimensions
field_size_x = 20;
field_size_y = 15;

% Grid Resolution
x_grid = linspace(0, field_size_x, 40);
y_grid = linspace(0, field_size_y, 30);
[terrain_map.x, terrain_map.y] = meshgrid(x_grid, y_grid);

% Base Elevation with Smooth Variation
base_elevation = 0.2 * sin(terrain_map.x/5) .* cos(terrain_map.y/4);

% Furrows: Adding structured periodic dips
furrow_pattern = 0.4 * sin(2 * pi * terrain_map.x / 1.5); % Furrows every 1.5 meters along x-axis

% Randomness for Ruts: More focused and deeper than the initial random variation
% Assuming ruts are made along specific paths, not randomly scattered
rut_paths = abs(sin(2 * pi * terrain_map.y / 5)) < 0.1; % Simulated paths for equipment
random_ruts = 0.5 * randn(size(terrain_map.x)) .* rut_paths; % Ruts are deeper and localized

% Combined Elevation
terrain_map.elevation = base_elevation + furrow_pattern + random_ruts;

% Friction: Adjusting for different soil moisture in furrows
% Assuming furrows (wetter areas) have slightly lower friction
terrain_map.friction = 0.8 + 0.15 * cos(2*terrain_map.x/field_size_x) - 0.1 * abs(sin(2 * pi * terrain_map.x / 1.5));


% Set up scenario
x0 = [0, 0, pi/2, 0];  
goal = [10,10];       
obstacles = [3, 6, 0.5;
             6, 8, 0.5]; 
N = 40;
dt = 0.8;

% Run MPC
[optimal_u, predicted_trajectory] = agricultural_MPC2(x0, goal, obstacles, N, dt, terrain_map);

valid_indices = find(predicted_trajectory(:,1) ~= 0 | predicted_trajectory(:,2) ~= 0 | predicted_trajectory(:,3) ~= 0 | predicted_trajectory(:,4) ~= 0);

% If there are no valid indices (all trajectory points are zero), we should avoid an error
if isempty(valid_indices)
    valid_end = 0;
else
    valid_end = max(valid_indices);
end

% Plot the terrain
figure;
surf(terrain_map.x, terrain_map.y, terrain_map.elevation, 'EdgeColor', 'none'); 
colorbar;
hold on;
% % Plot the predicted trajectory
% plot3(predicted_trajectory(:,1), predicted_trajectory(:,2), predicted_trajectory(:,4)+0.75, 'b-', 'LineWidth', 2); 

% Plot only the valid part of the trajectory
plot3(predicted_trajectory(1:valid_end,1), predicted_trajectory(1:valid_end,2), predicted_trajectory(1:valid_end,4)+0.75, 'b-', 'LineWidth', 2);

% Plot the goal
plot3(goal(1), goal(2), 0.75, 'ro', 'MarkerSize', 10, 'LineWidth', 2);
% Plot obstacles
% Plot obstacles with their size (radius)
for i = 1:size(obstacles, 1)
    obs = obstacles(i, :);
    viscircles([obs(1), obs(2)], obs(3), 'Color', 'k', 'LineWidth', 1);
    % Adjust the Z-coordinate of the circle to match your plotting needs
    % Note: viscircles does not support 3D plotting directly, so circles will appear on the XY plane.
end

hold off;

xlabel('X');
ylabel('Y');
zlabel('Elevation');
title('MPC Trajectory with Terrain');


