function [error, pe]=DirKin_planar_3DOF(qinput,L,qmin,qmax)    % this function implement the direct kinematics of planar robot with 3DOF

                                              % inputs:
                                              % qinput=[theta1 theta2 theta3] - vetor with the joint values (rad)
                                              % L=[L1 L2 L3] - vector with the two links lenghts (mm)
                                         
                                              % output:
                                              % pe=[xe ye] - vetor with desired values for the end-effector position

error = 0;

for ptr = 1:length(qinput)

    if qinput(ptr) > qmax(ptr) || qinput(ptr) < qmin(ptr)
        error = 1;   
        error('Angle is outside joint limits!'); %#ok<NOEFF>
    end
end

if qinput(1) < 0
    theta_1_u = (pi/2 - abs(qinput(1)));
else
    theta_1_u = pi/2 + qinput(1);
end

disp(rad2deg(qinput(1)));
disp(rad2deg(theta_1_u));


q = [theta_1_u(1),qinput(2),qinput(3)]; %Joint J1 is vertical, so that angle affects to Y axis instead of X, do this, we shift the quadrant
xe=L(1)*cos(q(1))+L(2)*cos(q(1)+q(2))+L(3)*cos(q(1)+q(2)+q(3));
ye=L(1)*sin(q(1))+L(2)*sin(q(1)+q(2))+L(3)*sin(q(1)+q(2)+q(3));

pe=[xe ye]';

end