function [error_inv_kin, q]=InvKin_planar_2DOF_geo(pe,L, S, qmin, qmax)
% function [q]=InvKin_planar_2DOF_geo(pe,L, S, qmin, qmax)
% this function implements the inverse kinematics of a planar robot with 2DOD
% inputs:
%   pe =[xe ye] - vector with desired values for the end-effector position
%   L=[L1 L2] - vector with the two links lengths (mm)
%   S - flag the signal the desired solution
%       S = 1 ---> Elbow right
%       S =-1 ---> Elbow left
%   qmin = [theta1_min theta2_min]  % joint limits minimum values
%   qmax = [theta1_max theta2_max]  % joint limits maximum values
%
% output:
%   q=[theta1 theta2] - vector with the joint values (rad)
%   error_inv_kin  = 0 - if solution exists
%                  = 1 - if there is no solution
%  Aula:  08/11/2023 MAERO

%Your code here:

error_inv_kin = 0;
%desired coordinates for the end-effector position
xed=pe(1);
yed=pe(2);

% ------ Compute Theta_1:
alfa1 = atan2(yed,xed);
%alfa1 = atan2(xed,yed);
r = sqrt(xed^2 + yed^2);
arg1 = (r^2+L(1)^2-L(2)^2)/(2*L(1)*r);

if arg1 >=-1 && arg1 <=1
   beta1 = acos(arg1);
else
   error_inv_kin = 1; %#ok<*NASGU>
   error('There is no solution for Theta1!');
end    

%Theta_1 is bound the Y axis intead of X, due to join1 which is vertical,
%here we change it's quadrant
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
