

L1 = 15.50;
L2 = 13.50;
L3 = 21.750;
L = [L1 L2 L3];

arm_joints = [deg2rad(-30),deg2rad(-40),deg2rad(-20)];
qmin = [-1.5708,   -2.2864,   -1.7802];
qmax = [1.3090,     2.2864,    1.7802];

p = kuka_direct_kinematics(arm_joints,L);
disp(p);

[error, angle] = kuka_inverse_kinematics(p,L,-1,qmin,qmax,pi/2);
disp(rad2deg(angle(1)));
disp(rad2deg(angle(2)));
disp(rad2deg(angle(3)));


p1 = kuka_direct_kinematics(angle,L);
disp(p1);