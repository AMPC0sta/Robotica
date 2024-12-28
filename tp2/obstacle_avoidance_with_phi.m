function [f_obstacle] = obstacle_avoidance_with_phi(phi,delta_theta,theta_obs,beta_1,beta_2,dist,rob_W,rob_L)

    for i = 1:length(dist)
        lambda_obs_i(i) = beta_1 * exp(-dist(i)/beta_2); %#ok<*AGROW>
        sigma_i(i) = atan(tan(delta_theta/2)+((rob_W/2)/(rob_L/2)+dist(i)));
    end
    
    for ptr = 1:length(phi)
        psi = phi(ptr) + theta_obs;
        part = 0;
        for i = 1:11
            angle = phi(ptr) - psi(i);
            disp(angle);
            f_component_i = lambda_obs_i(i) * angle * exp((-angle^2)./(2*(sigma_i(i))^2));
            part = part + f_component_i;
        end
       
        tmp_f_obstacle(ptr) = part;
    end
    disp (tmp_f_obstacle);
    f_obstacle = tmp_f_obstacle;
