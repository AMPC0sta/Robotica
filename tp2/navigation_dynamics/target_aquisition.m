function [f_target] = target_aquisition(phi,psi_tar,lambda_tar) 
% function type: Navigation Dynamics
% 
% Objective: Determine target direction angle and attractive force to target
% 
% Input:
%       phi             - Robot angle in the world frame
%       psi_tar         - Target angle in the world frame
%       lambda_tar      - Attractive force upper limit
% 
% Output
%       f_target        - Attractive force to target

    f_target = - lambda_tar * sin(phi - psi_tar);
end
