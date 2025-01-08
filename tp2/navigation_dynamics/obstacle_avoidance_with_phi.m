function [f_obstacle] = obstacle_avoidance_with_phi(phi,delta_theta,theta_obs,beta_1,beta_2,dist,rob_W,rob_L)
% function type: Navigation Dynamics
% 
% Objective: Determine obstacles directions and repulsive forces to it, to be used in graphical plotings of equations dynamics, needs a different version of 
%           the function, since usualy robot position to the world is not necessary in the regular dynamics, but this is need to plot equation curve.
%           
% Input:
%       phi             - Angle of the robot (will be used in sweeping from -2pi to 2pi)
%       delta_theta     - Angle of one LIDAR sector 
%       theta_obs       - LIDAR angles for each slice.
%       beta_1          - Obstacle avoidance parameter (maximum repulsive force)
%       beta_2          - Obstacle avoidance parameter (where repulsion starts to act on the robot)
%       dist            - LIDAR readings
%       rob_L           - Robot lenght
%       rob_W           - Robot width
% Output
%       f_target        - Repulsive force to obstacles

    for i = 1:length(dist)
        lambda_obs_i(i) = beta_1 * exp(-dist(i)/beta_2); %#ok<*AGROW>
        sigma_i(i) = atan(tan(delta_theta/2)+((rob_W/2)/(rob_L/2)+dist(i)));
    end
    
    for ptr = 1:length(phi)
        psi = phi(ptr) + theta_obs;
        part = 0;
        for i = 1:11
            angle = phi(ptr) - psi(i);
            %disp(angle);
            f_component_i = lambda_obs_i(i) * angle * exp((-angle^2)./(2*(sigma_i(i))^2));
            part = part + f_component_i;
        end
       
        tmp_f_obstacle(ptr) = part;
    end
    %disp (tmp_f_obstacle);
    f_obstacle = tmp_f_obstacle;
