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

% Set the font size
fontSize = 14; % You can adjust this value according to your needs
fontSize2 = 20;

% Plotting the elevation
figure;
surf(terrain_map.x, terrain_map.y, terrain_map.elevation);
title('Terrain Elevation', 'FontSize', fontSize2);
xlabel('X Coordinate', 'FontSize', fontSize);
ylabel('Y Coordinate', 'FontSize', fontSize);
zlabel('Elevation (m)', 'FontSize', fontSize);
colorbar('FontSize', fontSize);
shading interp;  % Smooths the color transitions

% Plotting the friction
figure;
imagesc(x_grid, y_grid, terrain_map.friction);
title('Terrain Friction', 'FontSize', fontSize2);
xlabel('X Coordinate', 'FontSize', fontSize);
ylabel('Y Coordinate', 'FontSize', fontSize);
colorbar('FontSize', fontSize);
axis equal;  % Maintain aspect ratio
