function [R] = kuka_direct_kinematics(arm_position, L)

    % inputs:
    % qinput=[theta1 theta2 theta3] - vetor with the joint values (rad)
    % L=[L1 L2 L3] - vector with the two links lenghts (mm)
    % output:
    % R=[xe ye] - vetor with desired values for the end-effector position
    q = [pi/2 - arm_position(1),arm_position(2),arm_position(3)]; %Joint J1 is vertical, so that angle affects to Y axis instead of X, do this, we shift the quadrant
    
    
    xe=L(1)*cos(q(1))+L(2)*cos(q(1)+q(2))+L(3)*cos(q(1)+q(2)+q(3));
    ye=L(1)*sin(q(1))+L(2)*sin(q(1)+q(2))+L(3)*sin(q(1)+q(2)+q(3));
                                              
    R=[xe ye]';                                           
end    
