clear all; %#ok<CLALL>

addpath('D:\Projects\personal\Robotica\Robotica\tp2');

L1 = 15.50;
L2 = 13.50;
L3 = 21.750;
L = [L1 L2 L3];

qmin = [-1.5708,   -2.2864,   -1.7802];
qmax = [1.3090,     2.2864,    1.7802];


armJoints = [deg2rad(30),deg2rad(30),deg2rad(30)];
[error, p] = DirKin_planar_3DOF(armJoints,L,qmin,qmax);

disp(p);
[error, angle] = InvKin_planar_3DOF_geo(p,L,1,qmin,qmax,pi/2);

disp(rad2deg(angle));

[error, p1]= DirKin_planar_3DOF(angle,L,qmin,qmax);
disp(p1);
