% Before running this script, open the scenario in CoppeliaSim, e.g
% Do not run simulation!
%   Author: Luís Louro, llouro@dei.uminho.pt
%           Estela Bicho, estela.bicho@dei.uminho.pt
%   % Copyright (C) 2023
%   2023/10/31
TARGET1 = 1;
TARGET2 = 2;
BOX = 3;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Creation of a communication class object with the simulator
% Try to connect to simulator
[sim,error_sim] = simulator_interface();
%Output:
% sim - pointer to class simulator_interface
% error_sim = 1 - impossible to connect to simulator
if(error_sim==1)
    return;
end
% Creation of a communication class object with the robot
[vehicle,error_kuka] = youBot_interface(sim);
%Input:
% sim - pointer to class simulator_interface
%Output:
% vehicle - pointer to class youBot_interface
% error_kuka = 1 - impossible to connect to youBot_interface
if(error_kuka==1)
    return;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%This function returns relevant information about the mobile platform 
[error,rob_W,rob_L,theta_obs] = vehicle.get_RobotCharacteristics();
%Output:
% error = 1 - error in function
% rob_W - robot width (cm)
% rob_L - robot lenght (cm)
% theta_obs - Vector with angle value for each sector of obstacle dynamic 
% i (i = 1, . . . , 11) relative to the frontal direction (in rad)
if(error==1)
    return;
end

%This function returns relevant information about the robotic manipulator
[error,nJoints,Links,MinPositionJoint,MaxPositionJoint] = ...
    vehicle.get_ManipulatorCharacteristics();
%Output:
%error = 1 - error in function
%nJoints - number of arm joints.
%Links - dimensions of the links between the axes of rotation (m)
%MinPositionJoint - array with minimum position for joint 1-5 (rad)
%MaxPositionJoint - array with maximum position for joint 1-5 (rad)
if error == 1
    sim.terminate();
    return;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initialize the manipulator in park position (arm (0,30,50,70,0) and
% gripper opened
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

armJoints(1)=0*pi/180;  
armJoints(2)=30*pi/180;
armJoints(3)=50*pi/180;
armJoints(4)=70*pi/180;
armJoints(5)=0*pi/180;
%This function send value for arm Joints in rad
error = vehicle.set_joints(armJoints); % in rad
%Input
%armJoints - Array with the new values to be applied to the arm joints
%Output:
%error = 1 - error in function
if error == 1
    sim.terminate();
    return;
end

%Function to open the gripper
error = vehicle.open_hand(); 
%Output:
%error = 1 - error in function
if error == 1
    sim.terminate();
    return;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Initialize the mobile platform with zero speeds
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
vrobot_y = 0;     %cm/s
vrobot_x = 20;       %cm/s
wrobot = 0.0;       %rad/s
error = 0;
vel_front_left = 0.0;   %rad/s
vel_front_right = 0.0;  %rad/s
vel_rear_left = 0.0;    %rad/s
vel_rear_right = 0.0;   %rad/s

% Convert longitudinal speed, lateral speed and angular speed into wheel 
% speed
[error,vel_front_left,vel_front_right,vel_rear_left,vel_rear_right] = ...
    vehicle.Kinematics_vehicle(wrobot, vrobot_y,vrobot_x);
%Input:
% wrobot - angular speed
% vrobot_x - longitudinal speed (cm/s)
% vrobot_y - lateral speed (cm/s)
%Output:
% error = 1 - error in function
% vel_front_left - front left wheel rotation speed (rad/s)
% vel_front_right - front right wheel rotation speed (rad/s)
% vel_rear_left - rear left wheel rotation speed (rad/s)
% vel_rear_right - rear right wheel rotation speed (rad/s)
if(error==1)
    return;
end

% Set wheels speeds
[error,~, ~, phi] = vehicle.set_velocity(vel_front_left,vel_front_right,...
    vel_rear_left,vel_rear_right);
%Input:
% vel_front_left - rad/s
% vel_front_right - rad/s
% vel_rear_left - rad/s
% vel_rear_right - rad/s
%Output:
% robot  (xrobot,yrobot) in cm
% phirobot - rad
% error = 1 - error in function
if(error==1)
    return;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Get time step value (normally dt=50ms)
[error,timestep] = sim.get_simulation_timestep();
%Output:
% timestep - information of time step (dt)
% error = 1 - error in function
if(error==1)
    return;
end

itarget=TARGET1;          %initialize first target
start = tic;
iteration = 0;

% Initializations out of loop:
vrobot_x = 50;
vrobot_y = 0;
wrobot = 0;

%dt = sim.get_simulation_time();
    
dt = 0.05;
tau_tar = 15 * dt;          % computation cycle, constant  will  change force magnitude
lambda_tar = 1 / tau_tar;   % atractor lamda
Q = 0.05;                   % base for gaussian noise


[~,rob_W,rob_L,theta_obs] = vehicle.get_RobotCharacteristics();

%beta_1 = 1/15*dt;
beta_1 = 150;
beta_2 = 30;


%%%---------------------- Start Robot Motion Behavior -------------------
while itarget<=sim.TARGET_Number % until robot goes to last target
    %% Robot interface
    % set and get information to/from CoppeliaSim
    % avoid do processing in between ensure_all_data and trigger_simulation
    sim.ensure_all_data();
    
    % Convert longitudinal speed, lateral speed and angular speed into wheel 
    % speed
    [error,vel_front_left,vel_front_right,vel_rear_left,vel_rear_right] = ...
        vehicle.Kinematics_vehicle(wrobot, vrobot_y,vrobot_x);
    %Input:
    % wrobot - angular speed
    % vrobot_x - longitudinal speed (cm/s)
    % vrobot_y - lateral speed (cm/s)
    %Output:
    % error = 1 - error in function
    % vel_front_left - front left wheel rotation speed (rad/s)
    % vel_front_right - front right wheel rotation speed (rad/s)
    % vel_rear_left - rear left wheel rotation speed (rad/s)
    % vel_rear_right - rear right wheel rotation speed (rad/s)
    if(error==1)
        return;
    end

    % Set wheels speeds
    % get also pose. it can be used vehicle.get_vehicle_pose() instead
    [error,x, y, phi] = vehicle.set_velocity(vel_front_left,...
        vel_front_right,vel_rear_left,vel_rear_right);
	%Input:
	% vel_front_left - rad/s
	% vel_front_right - rad/s
	% vel_rear_left - rad/s
	% vel_rear_right - rad/s
	%Output:
	% robot  (xrobot,yrobot) in cm
	% phirobot - rad
	% error = 1 - error in function
    if(error==1)
        return;
    end

    % trigger obstacles data reception
    error = vehicle.trigger_obstacles();
    % error = 1 - error in function
    if(error==1)
        return;
    end

    [error,ReadArmJoints] = vehicle.get_joints();
    %Output:
    % ReadArmJoints - joints values in rad
    % error = 1 - error in function
    if error == 1
        sim.terminate();
        return;
    end

    %Get target position for target 1 or 2 (itarget) or 3 for box
    [error,XTARGET,YTARGET] = sim.get_TargetPosition(BOX);
    %Input:
    %itarget - selection of target to get location (TARGET1,TARGET2,BOX)
    %Output:
    %XTARGET - location information on the x-axis of the target (cm)
    %YTARGET - location information on the y-axis of the target (cm)
    % error = 1 - error in function
    if(error==1)
        return;
    end
    
    %Get simulation time
    [error,sim_time] = sim.get_simulation_time();
    %Output:
    % sim_time - information of simulation time
    % error = 1 - error in function
    if(error==1)
        return;
    end

    %trigger simulation step
    sim.trigger_simulation();
   
     %% Processing step
    % Obtain the distances of the sectors of interest in obstacle avoidance 
    %dynamics.
    [error,dist] = vehicle.get_DistanceSensorAquisition(true, false);
    %Output:
	% dist - array of distances for each sector of obstacle avoidance 
    %dynamics (cm)
    % error = 1 - error in function
    if(error==1)
        return;
    end
    
    %%----------- BEGIN YOUR CODE HERE ----------- %

    % %Send values to arm joints
    % armJoints(1)=180*pi/180;  
    % armJoints(2)=60*pi/180;
    % armJoints(3)=70*pi/180;
    % armJoints(4)=50*pi/180;
    % armJoints(5)=0*pi/180;
    % %This function send value for arm Joints in rad
    % error = vehicle.set_joints(armJoints); % in rad
    % %Input
    % %armJoints - Array with the new values to be applied to the arm joints
    % %Output:
    % %error = 1 - error in function
    % if error == 1
    %    sim.terminate();
    %    return;
    % end

    % %Functions that allows open/close the hand
    %error = vehicle.open_hand(); 
    %error = vehicle.close_hand(); 
    % %Output:
    % %error = 1 - error in function
    %if error == 1
    %    sim.terminate();
    %    return;
    %end

    


 
    [~,XTARGET,YTARGET] = sim.get_TargetPosition(TARGET1);
    psi_tar = atan2 ((YTARGET - y),(XTARGET - x));
    
    
    %f_tar = - lambda_tar * sin(phi - psi_tar);
    f_tar = target_aquisition(phi,psi_tar,lambda_tar);
        

    delta_theta = theta_obs(2) - theta_obs(1); %sector width in radians
    %[~,dist] = vehicle.get_DistanceSensorAquisition(true, false);
    
    f_obs = obstacle_avoidance(delta_theta,theta_obs,beta_1,beta_2,dist,rob_L,rob_W);
    
    f_stoch = sqrt(Q) * randn(1,1);     % randn returns a 1x1 matrix with probility around a guassian distribution
    
    f_total = f_obs + f_tar + f_stoch;
    
    wrobot = f_total;
    
    %[~,v1,v2,v3,v4] = vehicle.Kinematics_vehicle(wrobot,vrobot_y,vrobot_x);
    %[error,x, y, phi] = vehicle.set_velocity(v1,v2,v3,v4);
    
    graphic_dynamics_view = 0;
    if graphic_dynamics_view == 1        
            phi_range = linspace(-2*pi,2*pi,25);
            f_tar_1 = target_aquisition(phi_range,psi_tar,lambda_tar);
            f_obs_1 = obstacle_avoidance_with_phi(phi_range,delta_theta,theta_obs,beta_1,beta_2,dist,rob_W,rob_L);
            f_t = f_obs_1 + f_tar_1 + f_stoch;
            plot(phi_range,f_t, 'b', 'LineWidth', 2);
            hold on
                %plot(phi, f_obs +  target_aquisition(phi,psi_tar,lambda_tar) + f_stoch, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r'); % Red point
                xline(phi, '--b', 'LineWidth', 2); % Creates a vertical line at each value in phi, in red color
                plot(phi_range,f_tar_1, 'g', 'LineWidth', 2);
                plot(phi_range,f_obs_1, 'r', 'LineWidth', 2);
            hold off
            ylim([-50,25]);
            % Add labels and title
            xlabel('Phi (radians)');
            ylabel('f_{tar} (Value)');
            title('Robot Motion Navigation Dynamics');
    
            lgd = legend("Total Force without stochastic force", "-- Current phi angle","Target Acquisition contribution", "Obstacle avoidance contribution");
    
            % Set colors for each legend entry
            lgd.TextColor = 'black';  % Set a default color for all entries if desired
    
            % Adjust individual colors (MATLAB does not natively support different colors in a single legend, 
            % but you can create custom colored legend icons if needed)
            lgd.String = {'\color{blue} Total Force without stochastic force','\color{blue} Current phi angle', '\color{green} Target Acquisition contribution','\color{red} Obstacle avoidance contribution'};
    
    end 


    %%------------- END OF YOUR CODE -------------
    
    %%----------------------------------------------------------------------
    %It allows to guarantee a minimum cycle time of 50ms for the 
    %computation cycle in Matlab
    time = toc(start);
    if time<0.05
        pause(0.05-time);
    else
        pause(0.01);
    end
    start = tic;
    iteration = iteration + 1;
    %----------------------------------------------------------------------
end
sim.terminate();

    