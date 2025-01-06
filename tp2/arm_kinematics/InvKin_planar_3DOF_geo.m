function [error_inv_kin, q]=InvKin_planar_3DOF_geo(pe,L, S, qmin, qmax, alpha3)
% function [q]=InvKin_planar_2DOF_geo(pe,L, S, qmin, qmax)
% this function implements the inverse kinematics of a planar robot with 2DOD
% inputs:
%   pe =[xe ye] - vector with desired values for the end-effector position
%   L=[L1 L2 L3] - vector with the two links lengths (mm)
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

x3 = xed - L(3)*sin(alpha3);
y3 = yed - L(3)*cos(alpha3);
%x3 = xed - L(3)*cos(alpha3);
%y3 = yed - L(3)*sin(alpha3);

%Now this turned on the 2DOF Inv Kin problem
[error,qout] = InvKin_planar_2DOF_geo([x3,y3],[L(1),L(2)], S, [qmin(1),qmin(2)], [qmax(1),qmax(2)]);
Theta_3 = alpha3 - qout(1) - qout(2);


if (Theta_3 <= qmin(3) || Theta_3 >= qmax(3))
   error_inv_kin = 1;
   error('Theta3 is outside joint limits!');
end

q=[qout(1),qout(2), Theta_3]';
end
