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

itarget=BOX;          %initialize first target
start = tic;
iteration = 0;


% Our initializations code here:
% This don't need to be inside the loop
CRUISE_VELOCITY = 100;
KINEMATICS_VELOCITY =30;

vrobot_x = CRUISE_VELOCITY;
vrobot_y = 0;
wrobot = 0;

%dt = sim.get_simulation_time();
    
dt = 0.05;
tau_tar = 5 * dt;           % computation cycle, constant  will  change force magnitude
lambda_tar = 2;             % small attractive force, to ensure repulsion is much higher
Q = 0.05;                   % base for gaussian noise


[~,rob_W,rob_L,theta_obs] = vehicle.get_RobotCharacteristics();

beta_1 = 10;               % repulsion higher than attraction
beta_2 = 100;              % As longest as repulsion is felt, as smooth is the path. However, to long will reach the walls.

itarget = TARGET1;         % set first target
ZTARGET = 2;               % box has 2 cm height

L = [Links(5)*100,Links(6)*100,(Links(7)+Links(8))*100];                    % Planar 3DOF links lenght, converted from m to cm (x100)
q_max = [MaxPositionJoint(2),MaxPositionJoint(3),MaxPositionJoint(4)];      % Planar 3DOF joints bounds
q_min = [MinPositionJoint(2),MinPositionJoint(3),MinPositionJoint(4)];
ARM_LENGHT = (Links(5) + Links(6) + Links(7) + Links(8)) * 100;             % maximum lenght arm can reach if scratched

joints_slack = 0.01;        % tolerance for joints slacks;

% State machine to rule the operation, MOVE -> GRASP -> PICK -> MOVE -> GRASP -> DROP
MOVE = 1;
GRASP = 2;
PICK = 3;
DROP = 4;

% State od the hand
OPENED_HAND = 1000;
CLOSED_HAND = 1001;

RIGHT = 1;                  % elbows
LEFT = -1;

action = MOVE;              % Status of the state machine, start operation in movement (reaching target1)
hand = OPENED_HAND;         % Status of the state machine, start operation with opened hand

OK = 0;                     % alias for human code reading (turn reading more natural)
NOK = 1;

% Modularity, adding each "module" paths
addpath('D:\Projects\personal\Robotica\Robotica\tp2\arm_kinematics');
addpath('D:\Projects\personal\Robotica\Robotica\tp2\navigation_dynamics');
addpath('D:\Projects\personal\Robotica\Robotica\tp2\auxiliaries');

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
    [error,XTARGET,YTARGET] = sim.get_TargetPosition(itarget);
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
 
    if action == MOVE
        [~,XTARGET,YTARGET] = sim.get_TargetPosition(itarget);
        psi_tar = atan2 ((YTARGET - y),(XTARGET - x));
        distance_robot_2_target = sqrt((XTARGET-x)^2+(YTARGET-y)^2);

        f_tar = target_aquisition(phi,psi_tar,lambda_tar);
        delta_theta = theta_obs(2) - theta_obs(1); %sector width in radians
        f_obs = obstacle_avoidance(delta_theta,theta_obs,beta_1,beta_2,dist,rob_L,rob_W);
        f_stoch = sqrt(Q) * randn(1,1);     % randn returns a 1x1 matrix with probability around a guassian distribution

        f_total = f_obs + f_tar + f_stoch;
        wrobot = f_total;
    
        x_cm = x + 15.4*cos(phi);
        y_cm = y + 15.4*sin(phi);

        z_cm = 25.015;              % Z box position
        alpha = 190*pi/180;         % arm orientation

        distance_arm_target = sqrt((XTARGET-x_cm)^2 + (YTARGET-y_cm)^2);

        if distance_arm_target > 10 && distance_arm_target < ARM_LENGHT             % if distance inside a certain area, reduce velocity
            vrobot_x = KINEMATICS_VELOCITY;
            xed = sqrt( (XTARGET-x_cm)^2 + (YTARGET-y_cm)^2);                       % transforms center of robot to arm base
            zed = ZTARGET-z_cm;

            try
                %[error, angles] = InvKin_planar_3DOF_geo([xed zed],L,RIGHT,q_min,q_max,alpha);      % do inverse kinematics while in move until one solution os possible.
                p = rotx(pi) * rotz(-pi/2) * [xed zed 0]';
                disp(p);
                [error, angles] = InvKin_planar_3DOF_geo([p(1) p(2)],L,RIGHT,q_min,q_max,alpha);      % do inverse kinematics while in move until one solution os possible.
            catch ME
                disp(ME.message);
                error = 1;
            end 

            if error == 0  && action == MOVE
                action = GRASP;
                vrobot_x = 0;
            end

        end
    end


    if action == GRASP 
        %[error, angles] = InvKin_planar_3DOF_geo([xed zed],L,RIGHT,q_min,q_max,alpha);             % Re-get the solution
            [error, angles] = InvKin_planar_3DOF_geo([p(1) p(2)],L,RIGHT,q_min,q_max,alpha);        % Re-get the solution
            theta0 = atan2((YTARGET - y_cm), (XTARGET - x_cm)) - phi;                               % align arm orientation to the target ( although in most scenarios wrobot != 0 before stoping ensure arm orientation firs joint will be pi)
        
            armJoints(1) =  theta0 + pi;
            error = vehicle.set_joints(armJoints); % in rad                                         % two phase movement: phase_1 => XY orientation, phase_2 XZ position (to ensure arm doesnt demage the robot itself)
  
            j = ReadArmJoints;
            result  = is_movement_complete(armJoints,ReadArmJoints,joints_slack);
            if result == OK && action == GRASP 
                wrobot = 0;
                arm_position = OK;
                armJoints(2) = angles(1);
                armJoints(3) = angles(2);
                armJoints(4) = angles(3);
                error = vehicle.set_joints(armJoints); % in rad  

                result1  = is_movement_complete(armJoints,ReadArmJoints,joints_slack);              % is robot picking the box? or dropping it?
                if result1 == OK && hand == OPENED_HAND
                    action = PICK;
                end

                if result1 == OK && hand == CLOSED_HAND
                    action = DROP;
                end
 
            end
    end

    if action == PICK
        %pause(1);
        wrobot = 0;
        vehicle.close_hand();
        sim.trigger_simulation();
        hand = CLOSED_HAND;

        armJoints(2)=30*pi/180;
        armJoints(3)=50*pi/180;
        armJoints(4)=70*pi/180;
        armJoints(5)=0*pi/180;
        error = vehicle.set_joints(armJoints); % in rad                                 two phase movement, phase_1 => Lift, phase_2 => XY re-orientation

        result1  = is_movement_complete(armJoints,ReadArmJoints,joints_slack/2);
        if result1 == OK
            armJoints(1)=0;
            error = vehicle.set_joints(armJoints); % in rad  
            itarget = TARGET2;
            vrobot_x = CRUISE_VELOCITY;
            action = MOVE;
        end
    end

    if action == DROP
        %pause(1);
        wrobot = 0;
        vehicle.open_hand();
        sim.trigger_simulation();
        hand = OPENED_HAND;
        
        armJoints(2)=30*pi/180;
        armJoints(3)=50*pi/180;
        armJoints(4)=70*pi/180;
        armJoints(5)=0*pi/180;
        error = vehicle.set_joints(armJoints); % in rad                                 two phase movement, phase_1 => Lift, phase_2 => XY re-orientation

        result1  = is_movement_complete(armJoints,ReadArmJoints,joints_slack/2);
        if result1 == OK
            armJoints(1)=0;
            error = vehicle.set_joints(armJoints); % in rad  
            result2  = is_movement_complete(armJoints,ReadArmJoints,joints_slack/2);
            if result2 == OK
                disp('Press any key to terminate...');
                pause;
                disp('Terminated!');
                break;
            end
        end
    end
    
    graphic_dynamics_view = 0;
    if graphic_dynamics_view == 1        
            phi_range = linspace(-2*pi,2*pi,25);
            f_tar_1 = target_aquisition(phi_range,psi_tar,lambda_tar);
           
            f_obs_1 = obstacle_avoidance_with_phi(phi_range,delta_theta,theta_obs,beta_1,beta_2,dist,rob_W,rob_L);
           
            f_t = f_obs_1 + f_tar_1 + f_stoch;
            ylim([-10,45]);

            
            plot(phi_range,f_t, 'b', 'LineWidth', 2);
            %plot(phi_range,f_t, 'b',phi_range,f_tar_1, 'g', phi_range,f_obs_1, 'r', 'LineWidth', 2);
            hold on
                %plot(phi, f_obs +  target_aquisition(phi,psi_tar,lambda_tar) + f_stoch, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r'); % Red point
                xline(phi, '--b', 'LineWidth', 2); % Creates a vertical line at each value in phi, in red color
                plot(phi_range,f_tar_1, 'g', 'LineWidth', 2);
                plot(phi_range,f_obs_1, 'r', 'LineWidth', 2);
            hold off
            
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

    