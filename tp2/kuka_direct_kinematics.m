function R = kuka_direct_kinematics(theta_1, theta_2, theta_3, Links)
    L0 = Links(1);      % center to arm
    L1 = Links(2);      % center to arm
    L2 = Links(3);      % center to arm
    L3 = Links(4);      % center to arm
    theta_0 = 0;

    x = L0*cos(theta_0) + L1*cos(theta_0 + theta_1) + L2*cos(theta_0 + theta_1 + theta_2) + L3*cos(theta_0 + theta_1 + theta_2 + theta_3);
    z = L0*sin(theta_0) + L1*sin(theta_0 + theta_1) + L2*sin(theta_0 + theta_1 + theta_2) + L3*sin(theta_0 + theta_1 + theta_2 + theta_3);
    
    R = [x,z];
