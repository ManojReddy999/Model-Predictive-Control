function U_seq = optimize_control_sequence(x0, goal, obstacles, N, dt, terrain_map)
    % Heuristic for initial guess: simple forward motion and small angular adjustments
    direction_to_goal = atan2(goal(2) - x0(2), goal(1) - x0(1)); % Angle to goal
    angular_diff = wrapToPi(direction_to_goal - x0(3)); % Angular difference to goal

    initial_speed = 0.2;  % Moderate initial speed
    initial_angular_velocity = 0.2*sign(angular_diff) * min(abs(angular_diff), pi/6); % Aim to turn towards the goal but within bounds;  % Small angular velocity
    U_initial = repmat([initial_speed; initial_angular_velocity], 1, N);  % Repeat for N steps

    % Define the cost function for the sequence
    cost_fn = @(U) total_cost(x0, U, goal, obstacles, N, dt, terrain_map);

    % Lower and upper bounds on linear/angular velocity for each time step
    lb = repmat([-3; -pi/3], 1, N);
    ub = repmat([3; pi/3], 1, N);

    % Optimization options
    options = optimoptions('fmincon', 'Algorithm', 'sqp', 'Display', 'off');

    % Run the optimizer to find the optimal sequence of controls
    U_seq = fmincon(cost_fn, U_initial, [], [], [], [], lb, ub, [], options);
end

function cost = total_cost(x, U, goal, obstacles, N, dt, terrain_map)
    cost = 0;
    for i = 1:N
        u = U(:, i);  % Extract control input for this step
        x = robot_dynamics2(x, u, dt, terrain_map);  % Simulate dynamics
        cost = cost + norm(goal(1:2) - x(1:2)) + 100*obstacle_proximity_cost2(x, obstacles);
    end
end
