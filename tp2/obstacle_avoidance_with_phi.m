function [f_obstacle] = obstacle_avoidance_with_phi(phi,delta_theta,theta_obs,beta_1,beta_2,dist,rob_W,rob_L)

    lambda_obs_i = beta_1 * exp(-dist/beta_2);
    sigma_i = atan(tan(delta_theta/2)+((rob_W/2)./(rob_L/2+dist)));
    phi_i = phi + theta_obs;

    part = 0;
    for i = 1:11
        %f_component_i = lambda_obs_i(i) .* (phi-phi_i(i)) .* exp((phi-phi_i(i)).^2/2.*(sigma_i(i)).^2);
        f_component_i = lambda_obs_i(i).*(phi_i(i) - theta_obs(i)).*exp(-(phi_i(i)-theta_obs(i)^2)./(2*sigma_i(i))^2);
        part = part + f_component_i;
    end

    f_obstacle = part;