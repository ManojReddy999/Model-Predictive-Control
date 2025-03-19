% function [optimal_u, predicted_trajectory] = agricultural_MPC2(x0, goal, obstacles, N, dt, terrain_map)
%     % Initialize predicted trajectory
%     predicted_trajectory = zeros(N, 4); 
%     predicted_trajectory(1,:) = x0;
% 
%     for i = 1:N-1
%         % Call `optimize_control` to get the optimal control input for the current state
%         u = optimize_control2(x0, goal, obstacles, dt, terrain_map); 
% 
%         % Apply control and simulate one step using `robot_dynamics`
%         x0 = robot_dynamics2(x0, u, dt, terrain_map); 
% 
%         % Store the state for visualization
%         predicted_trajectory(i+1,:) = x0;
% 
%         % Check if goal is reached (with some tolerance)
%         if norm(x0(1:2) - goal) < 0.5  
%             break;
%         end
%     end
% 
%     optimal_u = u; 
% end

function [optimal_u, predicted_trajectory] = agricultural_MPC2(x0, goal, obstacles, N, dt, terrain_map)
    % Initialize predicted trajectory
    predicted_trajectory = zeros(N, 4); 
    predicted_trajectory(1, :) = x0;

    for i = 1:N-1
        % Compute the control sequence for the current state
        U_seq = optimize_control_sequence(x0, goal, obstacles, N-i+1, dt, terrain_map);
        
        % Use the first control input as the optimal control
        u = U_seq(:, 1);
        optimal_u = u;  % Optionally, store all optimal_u if needed

        % Apply the control input and simulate one step using `robot_dynamics`
        x0 = robot_dynamics2(x0, u, dt, terrain_map); 

        % Store the state for visualization
        predicted_trajectory(i+1, :) = x0;

        % Check if goal is reached (with some tolerance)
        if norm(x0(1:2) - goal) < 0.5
            break;
        end
    end
end






