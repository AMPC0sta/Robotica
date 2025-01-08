function [error_inv_kin, q]=InvKin_planar_3DOF_geo(pe,L, S, qmin, qmax, alpha3)
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
%       alpha3          - Arm orientation
% 
% Output
%       q               - Joints position
error_inv_kin = 0;

x3 = pe(1) - L(3)*cos(alpha3);
y3 = pe(2) - L(3)*sin(alpha3);

%Now this turned on the 2DOF Inv Kin problem
[error,qout] = InvKin_planar_2DOF_geo([x3,y3],[L(1),L(2)], S, [qmin(1),qmin(2)], [qmax(1),qmax(2)]);
Theta_3 = alpha3 - qout(1) - qout(2);

if (Theta_3 <= qmin(3)  || Theta_3 >= qmax(3) )
   error_inv_kin = 1;
   error('Theta3 is outside joint limits!');
end

q=[qout(1),qout(2), Theta_3]';
end
