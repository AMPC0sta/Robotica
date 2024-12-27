function [f_obstacle] = obstacle_avoidance(delta_theta,theta_obs,beta_1,beta_2,dist,rob_L,rob_W)

    lambda_obs_i = beta_1 * exp(-dist/beta_2);

    disp(dist);
    sigma_i = atan(tan(delta_theta/2)+((rob_W/2)./((rob_L/2)+dist)));
    disp(sigma_i);


    partial = 0;
    for i = 1:11
        f_component_i = lambda_obs_i(i) * -theta_obs(i) * exp((-theta_obs(i)^2)/(2*(sigma_i(i))^2));
        partial = partial + f_component_i;
    end

    f_obstacle = partial;