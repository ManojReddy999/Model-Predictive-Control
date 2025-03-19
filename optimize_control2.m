function u = optimize_control2(x0, goal, obstacles, dt, terrain_map)
    % Heuristic for initial guess
    direction_to_goal = atan2(goal(2) - x0(2), goal(1) - x0(1)); % Angle to goal
    angular_diff = wrapToPi(direction_to_goal - x0(3)); % Angular difference to goal

    initial_speed = 0.2; % Moderate initial speed
    initial_angular_velocity = sign(angular_diff) * min(abs(angular_diff), pi/6); % Aim to turn towards the goal but within bounds

    % Define the cost function
    cost_fn = @(u) ...
        dt + ...
        norm(goal(1:2) - robotDynamicsPosition(x0, u, dt, terrain_map)) + ...
        obstacle_proximity_cost2(robotDynamicsPosition(x0, u, dt, terrain_map), obstacles);

    % Lower and upper bounds on linear/angular velocity
    lb = [-4; -pi/2];
    ub = [4; pi/2];

    % Optimization options, can be adjusted for performance or accuracy
    options = optimoptions('fmincon', 'Algorithm', 'sqp', 'Display', 'off');

    % Run the optimizer
    u = fmincon(cost_fn, [initial_speed; initial_angular_velocity], [], [], [], [], lb, ub, [], options); 
end

function pos = robotDynamicsPosition(x, u, dt, terrain_map)
    next_x = robot_dynamics2(x, u, dt, terrain_map);
    pos = next_x(1:2); % Extract only the position part of the state
end



