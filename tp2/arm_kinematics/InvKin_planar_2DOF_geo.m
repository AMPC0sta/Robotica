function [error_inv_kin, q]=InvKin_planar_2DOF_geo(pe,L, S, qmin, qmax)
% function type: Kinematics
% 
% Objective: return joints solution for 2DOF Planar Arm
% 
% Input:
%       pe              - End Effector Position
%       L               - Links
%       S               - Elbow position -1 (left) and 1 (right)
%       qmin            - Joints minimum boundaries
%       qmax            - Joints maximum boundaries
% 
% Output
%       q               - Joints position


error_inv_kin = 0;
%desired coordinates for the end-effector position
xed=pe(1);
yed=pe(2);

% ------ Compute Theta_1:
alfa1 = atan2(yed,xed);

r = sqrt(xed^2 + yed^2);
arg1 = (r^2+L(1)^2-L(2)^2)/(2*L(1)*r);

if arg1 >=-1 && arg1 <=1
   beta1 = acos(arg1);
else
   error_inv_kin = 1; %#ok<*NASGU>
   error('There is no solution for Theta1!');
end    

if S==1
   %Theta_1 = pi/2 - (alfa1-beta1);   % Elbow right
   Theta_1 = alfa1-beta1;
elseif S==-1
   %Theta_1 = pi/2 - (alfa1+beta1);   % Elbow left  
   Theta_1 = (beta1 + alfa1);
end


% ------ Compute Theta_2:
arg2 = (L(1)^2+L(2)^2-r^2)/(2*L(1)*L(2));
if arg2 >=-1 && arg2 <=1
   beta2 = acos(arg2);
else
   error_inv_kin = 1;
   error('Não há solucao para Theta2!');
end 
if S==1
   Theta_2 = pi-beta2;    % Elbow right
elseif S==-1
   Theta_2 = -(pi-beta2);   % Elbow left  
end


% check if Theta_1 is outside joint limits
if (Theta_1 < qmin(1)|| Theta_1 > qmax(1))
   error_inv_kin = 1;
   error('Theta1 is outside joint limits!');
end

% check if Theta_2 is outside joint limits
if (Theta_2 < qmin(2) || Theta_2 > qmax(2))
    error_inv_kin = 1;
    error('Theta2 is outside joint limits!');
end

q=[Theta_1, Theta_2]';
end
