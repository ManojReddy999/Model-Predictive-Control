function cost = obstacle_proximity_cost2(x, obstacles)
    cost = 0;
    for i = 1:size(obstacles, 1)
        obs = obstacles(i, :);  % Extract the i-th obstacle
        distance = norm(x(1:2) - obs(1:2));
        if distance < obs(3)  % Add buffer zone around obstacles
            cost = cost + 1/(distance^2);  % Higher cost for closer obstacles
        end
    end
end
