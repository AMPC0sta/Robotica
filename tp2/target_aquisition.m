function [f_target] = target_aquisition(phi,psi_tar,lambda_tar) 
    
    f_target = - lambda_tar * sin(phi - psi_tar);
end
