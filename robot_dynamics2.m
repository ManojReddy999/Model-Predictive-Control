function next_x = robot_dynamics2(x, u, dt, terrain_map)
    % Terrain effect - get terrain data
    current_terrain = interp2(terrain_map.x, terrain_map.y, terrain_map.friction, x(1), x(2), 'linear', 0);
    current_elevation = interp2(terrain_map.x, terrain_map.y, terrain_map.elevation, x(1), x(2), 'linear', x(4));

    % Modify linear velocity based on friction
    v_modified = u(1) * current_terrain;

    % Incline/Decline effect
    elevation_factor = 1 - 0.1 * abs(current_elevation - x(4)); 
    linear_motion_x = v_modified * cos(x(3)) * dt * elevation_factor;
    linear_motion_y = v_modified * sin(x(3)) * dt * elevation_factor;

    % Terrain-induced yaw disturbance
    yaw_disturbance = 0.5 * (current_elevation - x(4)) * sign(u(1));

    % Update state
    next_x = [
        x(1) + linear_motion_x,  
        x(2) + linear_motion_y,
        x(3) + u(2) * dt + yaw_disturbance,
        current_elevation  
    ];
end
