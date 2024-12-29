function R = kuka_inverse_kinematics_2dof(x,z,L1,L2)

    theta_1 = atan2(x,z) - acos((x^2 + y^2 + L1^2 - L2^2)/(2*L1*sqrt(x^2+z^2)));
    R = [theta_1];