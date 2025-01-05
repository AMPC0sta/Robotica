function [f_obstacle] = obstacle_avoidance(delta_theta,theta_obs,beta_1,beta_2,dist,rob_L,rob_W)
% function type: Navigation Dynamics
% 
% Objective: Determine obstacles directions and repulsive forces to it.
% 
% Input:
%       delta_theta     - Angle of one LIDAR sector 
%       theta_obs       - LIDAR angles for each slice.
%       beta_1          - Obstacle avoidance parameter (maximum repulsive force)
%       beta_2          - Obstacle avoidance parameter (where repulsion starts to act on the robot)
%       dist            - LIDAR readings
%       rob_L           - Robot lenght
%       rob_W           - Robot width
% Output
%       f_target        - Repulsive force to obstacles

    lambda_obs_i = beta_1 * exp(-dist/beta_2);

    
    sigma_i = atan(tan(delta_theta/2)+((rob_W/2)./((rob_L/2)+dist)));
    
    partial = 0;
    for i = 1:11
        f_component_i = lambda_obs_i(i) * -theta_obs(i) * exp(-(theta_obs(i)^2)/(2*(sigma_i(i))^2));
        partial = partial + f_component_i;
    end

    f_obstacle = partial;