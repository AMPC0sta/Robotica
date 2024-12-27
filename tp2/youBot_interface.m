classdef youBot_interface < handle
    %kuka_interface CoppeliaSim connection interface for a youBot Robot
    %   The class provides the communication service and information
    %   pre-processing for a MATLAB based control interface with a
    %   developed robot in CoppeliaSim.
    %   2023/10/20
    %   % Copyright (C) 2023, by Luís Louro
    %   Author: Luís Louro  e-mail: luislouro@algoritmi.uminho.pt

    properties (Access = private)
        vrep
        clientID
        motor_front_left_handle
        motor_front_right_handle
        motor_rear_left_handle
        motor_rear_right_handle
        S1_VS1_handle
        S1_VS2_handle

        jointHandle  % 1,2,... handle for each of the joints of the manipulator
        HandHandle   % handle for the gripper

        S1_VS1_ignore_steps
        S1_VS2_ignore_steps

        S1_maxDist

        unknown_sector_method

        world_ref_handle

        robot_shape_handle
        omni_ref_handle
        obstacles_lim
        nr_sectors
        theta_obs
        theta_obs_2pi
        x_sensor_final
        y_sensor_final
        b_sensor_i
        in_veh_dist
        robot_distance
        min_sensor
        max_sensor
        L
        W
        H
        displc
        %distance from steering wheel to the vehicle rotation center


        m0      %transform from WORLD frame to frame 0(robot rear axle center)
        m1      %transform from WORLD frame to frame 1(S1 VS1 frame)
        m2      %transform from WORLD frame to frame 2(S1 VS2 frame)
        m3      %transform from WORLD frame to frame 3(S2 VS1 frame)
        m4      %transform from WORLD frame to frame 4(S2 VS2frame)

        m1_pos
        m1_ori
        m2_pos
        m2_ori
        m3_pos
        m3_ori
        m4_pos
        m4_ori

        m01  %transform from frame 0 to frame 1
        m02  %transform from frame 0 to frame 2
        m03  %transform from frame 0 to frame 3
        m04  %transform from frame 0 to frame 4
    end

    properties
        name
        LONGITUDINAL_MAX_LINEAR_SPEED = 50  %cm/s;
        LATERAL_MAX_LINEAR_SPEED = 50       %cm/s ;
        wheel_radius = (0.1)/2;             %m
        lx = 0.22;                          %m
        ly = 0.15;                          %m
        auxDataS1VS1
        auxDataS1VS2
        DEFAULT_DETECTION_RANGE = 2*pi;

        number_of_joints  % variable indicating the number of joints of the manipulator arm
        robot_name        % variable indicating the name of the handler in CoppeliaSim
        MinJointPos       % vector indicating the minimum position allowed for the joint i
        MaxJointPos       % vector indicating the maximun position allowed for the joint i
    end

    methods
        % Create a connection handler to a SAMU Robot
        % sim_obj The simulation connection object
        function [obj,erro] = youBot_interface(sim_obj)
			erro = 0;
            [obj.vrep, obj.clientID] = sim_obj.get_connection();

            if obj.clientID <= -1
                clear obj;
                msg = 'ERROR: sim_obj seems to have an invalid connection to simulator!\n';
                error(msg)
            end

            %% Get objects handles
            obj.world_ref_handle = -1;

            [res, obj.robot_shape_handle] = obj.vrep.simxGetObjectHandle(obj.clientID,'/robot_bounding_box', obj.vrep.simx_opmode_blocking);
            if (res ~= obj.vrep.simx_return_ok)
                msg = 'ERROR: Failed getting robot_bounding_box handle ';
                error(msg);
            end

            [res, obj.motor_front_left_handle] = obj.vrep.simxGetObjectHandle(obj.clientID, './rollingJoint_fl', obj.vrep.simx_opmode_blocking);
            if (res ~= obj.vrep.simx_return_ok)
                msg = 'ERROR: Failed getting front left motor handle ';
                error(msg);
            end

            [res, obj.motor_front_right_handle] = obj.vrep.simxGetObjectHandle(obj.clientID,'./rollingJoint_fr', obj.vrep.simx_opmode_blocking);
            if (res ~= obj.vrep.simx_return_ok)
                msg = 'ERROR: Failed getting front right motor handle ';
                error(msg);
            end

            [res, obj.motor_rear_left_handle] = obj.vrep.simxGetObjectHandle(obj.clientID, './rollingJoint_rl', obj.vrep.simx_opmode_blocking);
            if (res ~= obj.vrep.simx_return_ok)
                msg = 'ERROR: Failed getting rear left motor handle ';
                error(msg);
            end

            [res, obj.motor_rear_right_handle] = obj.vrep.simxGetObjectHandle(obj.clientID,'./rollingJoint_rr', obj.vrep.simx_opmode_blocking);
            if (res ~= obj.vrep.simx_return_ok)
                msg = 'ERROR: Failed getting rear right motor handle ';
                error(msg);
            end


            [res, obj.S1_VS1_handle] = obj.vrep.simxGetObjectHandle(obj.clientID,'./SICK_TIM310_sensor1', obj.vrep.simx_opmode_blocking);
            if (res ~= obj.vrep.simx_return_ok)
                msg = 'ERROR: Failed getting SICK_TIM310_sensor1 handle ';
                error(msg);
            end

            [res, obj.S1_VS2_handle] = obj.vrep.simxGetObjectHandle(obj.clientID,'./SICK_TIM310_sensor2', obj.vrep.simx_opmode_blocking);
            if (res ~= obj.vrep.simx_return_ok)
                msg = 'ERROR: Failed getting SICK_TIM310_sensor2 handle ';
                error(msg);
            end


            [res, obj.omni_ref_handle] = obj.vrep.simxGetObjectHandle(obj.clientID,'/youBot_ref', obj.vrep.simx_opmode_blocking);
            if (res ~= obj.vrep.simx_return_ok)
                msg = 'ERROR: Failed getting omni_ref handle ';
                error(msg);
            end

            %% Parameters
            % Get vehicle size:
            localxMinMax = [0,0];
            localyMinMax = [0,0];
            localzMinMax = [0,0];
            [res, localxMinMax(1)] = obj.vrep.simxGetObjectFloatParameter(obj.clientID,obj.robot_shape_handle, obj.vrep.sim_objfloatparam_objbbox_min_x, obj.vrep.simx_opmode_blocking);
            if (res ~= obj.vrep.simx_return_ok)
                msg = 'ERROR: Failed to get robot size parameter';
                error(msg);
            end
            [res, localyMinMax(1)] = obj.vrep.simxGetObjectFloatParameter(obj.clientID,obj.robot_shape_handle, obj.vrep.sim_objfloatparam_objbbox_min_y, obj.vrep.simx_opmode_blocking);
            if (res ~= obj.vrep.simx_return_ok)
                msg = 'ERROR: Failed to get robot size parameter';
                error(msg);
            end
            [res, localzMinMax(1)] = obj.vrep.simxGetObjectFloatParameter(obj.clientID,obj.robot_shape_handle, obj.vrep.sim_objfloatparam_objbbox_min_z, obj.vrep.simx_opmode_blocking);
            if (res ~= obj.vrep.simx_return_ok)
                msg = 'ERROR: Failed to get robot size parameter';
                error(msg);
            end
            [res, localxMinMax(2)] = obj.vrep.simxGetObjectFloatParameter(obj.clientID,obj.robot_shape_handle, obj.vrep.sim_objfloatparam_objbbox_max_x, obj.vrep.simx_opmode_blocking);
            if (res ~= obj.vrep.simx_return_ok)
                msg = 'ERROR: Failed to get robot size parameter';
                error(msg);
            end
            [res, localyMinMax(2)] = obj.vrep.simxGetObjectFloatParameter(obj.clientID,obj.robot_shape_handle, obj.vrep.sim_objfloatparam_objbbox_max_y, obj.vrep.simx_opmode_blocking);
            if (res ~= obj.vrep.simx_return_ok)
                msg = 'ERROR: Failed to robot size parameter';
                error(msg);
            end
            [res, localzMinMax(2)] = obj.vrep.simxGetObjectFloatParameter(obj.clientID,obj.robot_shape_handle, obj.vrep.sim_objfloatparam_objbbox_max_z, obj.vrep.simx_opmode_blocking);
            if (res ~= obj.vrep.simx_return_ok)
                msg = 'ERROR: Failed to robot size parameter';
                error(msg);
            end
            xSize = localxMinMax(2) - localxMinMax(1);
            ySize = localyMinMax(2) - localyMinMax(1);
            zSize = localzMinMax(2) - localzMinMax(1);

            % get displacement in xx
            [res, pos] = obj.vrep.simxGetObjectPosition(obj.clientID, obj.robot_shape_handle, obj.omni_ref_handle, obj.vrep.simx_opmode_blocking);
            if (res ~= obj.vrep.simx_return_ok)
                msg = 'ERROR: Failed to robot size parameter';
                error(msg);
            end

            center_xx_disp = abs(pos(1));

            % ensure x size is the Len of the vehicle
            if xSize < ySize
                temp = ySize;
                ySize = xSize;
                xSize = temp;
            end

            obj.L = xSize;		%in m
            obj.W = ySize;		%in m
            obj.H = zSize;		%in m

            obj.displc = center_xx_disp;

            [res, obj.S1_maxDist] = obj.vrep.simxGetFloatSignal(obj.clientID,'SICK_TIM310_maxDist', obj.vrep.simx_opmode_blocking);
            if (res ~= obj.vrep.simx_return_ok)
                msg = 'ERROR: Failed to get max scan distance for SICK_TIM310';
                error(msg);
            end


            [res, obj.m1_pos] = obj.vrep.simxGetObjectPosition(obj.clientID, obj.S1_VS1_handle, obj.omni_ref_handle, obj.vrep.simx_opmode_blocking);
            if (res ~= obj.vrep.simx_return_ok)
                msg = 'ERROR: Failed to get object matrix';
                error(msg);
            end

            [res, obj.m1_ori] = obj.vrep.simxGetObjectOrientation(obj.clientID, obj.S1_VS1_handle, obj.omni_ref_handle, obj.vrep.simx_opmode_blocking);
            if (res ~= obj.vrep.simx_return_ok)
                msg = 'ERROR: Failed to get object matrix';
                error(msg);
            end

            [res, obj.m2_pos] = obj.vrep.simxGetObjectPosition(obj.clientID, obj.S1_VS2_handle, obj.omni_ref_handle, obj.vrep.simx_opmode_blocking);
            if (res ~= obj.vrep.simx_return_ok)
                msg = 'ERROR: Failed to get object matrix';
                error(msg);
            end

            [res, obj.m2_ori] = obj.vrep.simxGetObjectOrientation(obj.clientID, obj.S1_VS2_handle, obj.omni_ref_handle, obj.vrep.simx_opmode_blocking);
            if (res ~= obj.vrep.simx_return_ok)
                msg = 'ERROR: Failed to get object matrix';
                error(msg);
            end

            % get ignore steps for laser scanners
            [res, packed_ignore_steps] = obj.vrep.simxGetStringSignal(obj.clientID, 'SICK_TIM310_VS1', obj.vrep.simx_opmode_blocking);
            if (res ~= obj.vrep.simx_return_ok)
                msg = 'ERROR: Failed to get ignore steps for SICK_TIM310_VS1';
                error(msg);
            end

            ignore_steps = obj.vrep.simxUnpackInts(packed_ignore_steps);
            if ignore_steps(1) ~= 0
                obj.S1_VS1_ignore_steps = ignore_steps;
            end

            [res, packed_ignore_steps] = obj.vrep.simxGetStringSignal(obj.clientID, 'SICK_TIM310_VS2', obj.vrep.simx_opmode_blocking);
            if (res ~= obj.vrep.simx_return_ok)
                msg = 'ERROR: Failed to get ignore steps for SICK_TIM310_VS2';
                error(msg);
            end

            ignore_steps = obj.vrep.simxUnpackInts(packed_ignore_steps);
            if ignore_steps(1) ~= 0
                obj.S1_VS2_ignore_steps = ignore_steps;
            end


            %% Setup data streaming
            % simulation time
            [res, ~] = obj.vrep.simxGetFloatSignal(obj.clientID,'SimulationTime', obj.vrep.simx_opmode_streaming);
            if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                msg = 'ERROR: Failed to setup data streaming for simulation time';
                error(msg);
            end

            % Vision sensors for obstacles
            [res , ~, ~, ~] = obj.vrep.simxReadVisionSensor(obj.clientID, obj.S1_VS1_handle, obj.vrep.simx_opmode_streaming);
            if (res ~= obj.vrep.simx_return_ok  && res ~= obj.vrep.simx_return_novalue_flag)
                msg = 'ERROR: Failed to setup data streaming for S1_VS1_handle';
                error(msg);
            end

            [res , ~, ~, ~] = obj.vrep.simxReadVisionSensor(obj.clientID, obj.S1_VS2_handle, obj.vrep.simx_opmode_streaming);
            if (res ~= obj.vrep.simx_return_ok  && res ~= obj.vrep.simx_return_novalue_flag)
                msg = 'ERROR: Failed to setup data streaming for S1_VS2_handle';
                error(msg);
            end

            % Vehicle pose
            [res , ~] = obj.vrep.simxGetObjectPosition(obj.clientID, obj.omni_ref_handle, obj.world_ref_handle, obj.vrep.simx_opmode_streaming);
            if (res ~= obj.vrep.simx_return_ok  && res ~= obj.vrep.simx_return_novalue_flag)
                msg = 'ERROR: Failed to setup data streaming for robot position';
                error(msg);
            end
            [res , ~] = obj.vrep.simxGetObjectOrientation(obj.clientID, obj.omni_ref_handle, obj.world_ref_handle, obj.vrep.simx_opmode_streaming);
            if (res ~= obj.vrep.simx_return_ok  && res ~= obj.vrep.simx_return_novalue_flag)
                msg = 'ERROR: Failed to setup data streaming for robot orientation';
                error(msg);
            end

            % Get Joints velocities
            [res , ~] = obj.vrep.simxGetObjectVelocity(obj.clientID, obj.motor_front_left_handle, obj.vrep.simx_opmode_streaming);
            if (res ~= obj.vrep.simx_return_ok  && res ~= obj.vrep.simx_return_novalue_flag)
                msg = 'ERROR: Failed to setup data streaming for front left motor';
                error(msg);
            end

            [res , ~] = obj.vrep.simxGetObjectVelocity(obj.clientID, obj.motor_front_right_handle, obj.vrep.simx_opmode_streaming);
            if (res ~= obj.vrep.simx_return_ok  && res ~= obj.vrep.simx_return_novalue_flag)
                msg = 'ERROR: Failed to setup data streaming for front right motor';
                error(msg);
            end

            [res , ~] = obj.vrep.simxGetObjectVelocity(obj.clientID, obj.motor_rear_left_handle, obj.vrep.simx_opmode_streaming);
            if (res ~= obj.vrep.simx_return_ok  && res ~= obj.vrep.simx_return_novalue_flag)
                msg = 'ERROR: Failed to setup data streaming for rear left motor';
                error(msg);
            end

            [res , ~] = obj.vrep.simxGetObjectVelocity(obj.clientID, obj.motor_rear_right_handle, obj.vrep.simx_opmode_streaming);
            if (res ~= obj.vrep.simx_return_ok  && res ~= obj.vrep.simx_return_novalue_flag)
                msg = 'ERROR: Failed to setup data streaming for rear right motor';
                error(msg);
            end

            %arm joint handles
            jointName{1} = 'youBotArmJoint0';
            jointName{2} = 'youBotArmJoint1';
            jointName{3} = 'youBotArmJoint2';
            jointName{4} = 'youBotArmJoint3';
            jointName{5} = 'youBotArmJoint4';
            j=1;
            %obj.number_of_joints = 5;
            [~,obj.number_of_joints]=size(jointName);

            while j<=5
                [res,obj.jointHandle{j}]=obj.vrep.simxGetObjectHandle(obj.clientID,jointName{j},obj.vrep.simx_opmode_blocking);

                %obj.vrep.simxSetJointTargetVelocity (obj.clientID,obj.jointHandle{j},0*pi/180,obj.vrep.simx_opmode_oneshot);

                [res,~]=obj.vrep.simxGetJointPosition(obj.clientID, obj.jointHandle{j},obj.vrep.simx_opmode_streaming);
                if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                    disp('ERROR: Failed getting joint position information');
                    error = 1;
                    return;
                end
                j=j+1;
            end

            fingerName{1} = 'youBotGripperJoint1';
            fingerName{2} = 'youBotGripperJoint2';
            m=1;
            %Get Hand Handle
            while m<=2
                [res, obj.HandHandle{m}] = obj.vrep.simxGetObjectHandle(obj.clientID, fingerName{m}, obj.vrep.simx_opmode_blocking);
                if (res ~= obj.vrep.simx_return_ok)
                    disp('ERROR: Failed getting hand handle');
                    error = 1;
                    return;
                end
                m=m+1;
            end

        end

        function [error,vel_front_left,vel_front_right,vel_rear_left,vel_rear_right] = Kinematics_vehicle(obj, wrobot, vrobot_y, vrobot_x)
            error=0;
            if vrobot_x>obj.LONGITUDINAL_MAX_LINEAR_SPEED
                vrobot_x = 50;
            end
            if vrobot_y>obj.LATERAL_MAX_LINEAR_SPEED
                vrobot_y = 50;
            end

            v_robot_x_meters = vrobot_x*0.01;	%convert cm/s to m/s
            v_robot_y_meters = vrobot_y*0.01;	%convert cm/s to m/s

            vel_front_left=(v_robot_x_meters-v_robot_y_meters-(obj.lx+obj.ly)*wrobot)/obj.wheel_radius;
            vel_front_right=(v_robot_x_meters+v_robot_y_meters+(obj.lx+obj.ly)*wrobot)/obj.wheel_radius;
            vel_rear_left=(v_robot_x_meters+v_robot_y_meters-(obj.lx+obj.ly)*wrobot)/obj.wheel_radius;
            vel_rear_right=(v_robot_x_meters-v_robot_y_meters+(obj.lx+obj.ly)*wrobot)/obj.wheel_radius;
        end

        function  [erro, x, y, phi] = set_velocity(obj, vel_front_left, vel_front_right, vel_rear_left, vel_rear_right)
            erro = 0;
            %% Set vehicle velocity
            res = obj.vrep.simxSetJointTargetVelocity(obj.clientID, obj.motor_front_left_handle, -vel_front_left, obj.vrep.simx_opmode_oneshot);
            if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                msg = 'ERROR: Failed sending speed data! \n';
                erro = 1;
                error(msg);
                
            end

            res = obj.vrep.simxSetJointTargetVelocity(obj.clientID, obj.motor_front_right_handle, -vel_front_right, obj.vrep.simx_opmode_oneshot);
            if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                msg = 'ERROR: Failed sending speed data! \n';
                erro = 1;
                error(msg);
            end

            res = obj.vrep.simxSetJointTargetVelocity(obj.clientID, obj.motor_rear_left_handle, -vel_rear_left, obj.vrep.simx_opmode_oneshot);
            if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                msg = 'ERROR: Failed sending speed data! \n';
                erro = 1;
                error(msg);
            end

            res = obj.vrep.simxSetJointTargetVelocity(obj.clientID, obj.motor_rear_right_handle, -vel_rear_right, obj.vrep.simx_opmode_oneshot);
            if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                msg = 'ERROR: Failed sending speed data! \n';
                erro = 1;
                error(msg);
            end
            
            if erro==0
                % Get pose
                [erro, x, y, phi] = get_vehicle_pose(obj);
            end

        end

        function [erro, x, y, phi] = get_vehicle_pose(obj)
            erro = 0;
            % Get pose
            [res, robot_pos] = obj.vrep.simxGetObjectPosition(obj.clientID, obj.omni_ref_handle, obj.world_ref_handle, obj.vrep.simx_opmode_buffer);
            if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                msg = 'Failed getting robot position!';
				erro = 1;
                error(msg);
            end

            [res, robot_ori] = obj.vrep.simxGetObjectOrientation(obj.clientID, obj.omni_ref_handle, obj.world_ref_handle, obj.vrep.simx_opmode_buffer);
            if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                msg = 'Failed getting robot orientation!';
                error(msg);
                erro = 1;
            end

            x = robot_pos(1)*100;   %cm
            y = robot_pos(2)*100;   %cm
            phi = robot_ori(3);     %rad/s
        end

        function [x, y, phi] = get_vehicle_pose2pi(obj)
            [x, y, phi] = get_vehicle_pose(obj);
            phi = rem(phi + 2*pi, 2*pi);
        end

        % Get current power wheel velocities:
        % v_steer current linear velocity of power wheel
        % theta_steer current angular position of steering
        % w_theta_steer current velocity of steering
        function [erro,vel_front_left, vel_front_right, vel_rear_left, vel_rear_right] = get_current_state(obj)
            erro = 0
            % Get Joints velocities
            [res , vel_front_left] = obj.vrep.simxGetObjectVelocity(obj.clientID, obj.motor_front_left_handle, obj.vrep.simx_opmode_streaming);
            if (res ~= obj.vrep.simx_return_ok  && res ~= obj.vrep.simx_return_novalue_flag)
                msg = 'ERROR: Failed to setup data streaming for front left motor';
                error(msg);
                erro = 1;
            end

            [res , vel_front_right] = obj.vrep.simxGetObjectVelocity(obj.clientID, obj.motor_front_right_handle, obj.vrep.simx_opmode_streaming);
            if (res ~= obj.vrep.simx_return_ok  && res ~= obj.vrep.simx_return_novalue_flag)
                msg = 'ERROR: Failed to setup data streaming for front right motor';
                error(msg);
                erro = 1;
            end

            [res , vel_rear_left] = obj.vrep.simxGetObjectVelocity(obj.clientID, obj.motor_rear_left_handle, obj.vrep.simx_opmode_streaming);
            if (res ~= obj.vrep.simx_return_ok  && res ~= obj.vrep.simx_return_novalue_flag)
                msg = 'ERROR: Failed to setup data streaming for rear left motor';
                error(msg);
                erro = 1;
            end

            [res , vel_rear_right] = obj.vrep.simxGetObjectVelocity(obj.clientID, obj.motor_rear_right_handle, obj.vrep.simx_opmode_streaming);
            if (res ~= obj.vrep.simx_return_ok  && res ~= obj.vrep.simx_return_novalue_flag)
                msg = 'ERROR: Failed to setup data streaming for rear right motor';
                error(msg);
                erro = 1;
            end
        end

        function [error,rob_W,rob_L,theta_obs] = get_RobotCharacteristics(obj)
            % Setup number of sectors for obstacles detection.
            %   Set the method of dealing with occluded sectors (either 'inf' or 'max')
            error = 0;
            Nsensors = 11;
            range = (180+30.0)*pi/180;
            [error, theta_obs,delta_obs,in_veh_dist] = setup_obstacles(obj,Nsensors, range, 'inf');
            if error==0
                [error,rob_W, rob_L, rob_H] = get_robot_dimension(obj);
                %Convertido para cm no get_robot_dimension
            end
        end

        function [erro,theta_obs, delta_theta_obs, in_veh_dist] = setup_obstacles(obj, nr_sectors, detection_range, occluded_method)
            erro = 0;
            obj.unknown_sector_method = inf;
            if nargin >= 3
                if ~isfloat(detection_range)
                    error('detection_range paramter must be a double or single value, e.g. 2*pi. Leave empty for default');
                    erro = 1;
                end
                detection_range_ = detection_range;
            else
                detection_range_ = obj.DEFAULT_DETECTION_RANGE;
            end
            if nargin >= 3
                if strcmp(occluded_method, 'inf')
                    obj.unknown_sector_method = inf;
                elseif strcmp(occluded_method, 'max')
                    obj.unknown_sector_method = obj.S1_maxDist * 1.1;
                else
                    error(['unknown argument: ' occluded_method]);
                    erro = 1;
                end
            end

            % enforce some options
            if nr_sectors <= 0
                error('Invalid number of sectors. Value should be higher than 0');
            end
            if rem(nr_sectors, 2) == 0
                error('Invalid number of sectors. Value should be odd');
            end

            % angular difference between to sectors
            % consider detetion range to be the full circle, or given
            % parameter
            %detection_range_ = 2*pi;   % see parameter and default value
            delta_theta_obs = detection_range_/nr_sectors;
            obj.nr_sectors = nr_sectors;

            %store sectors angles limits (start angle and end angle )
            obj.obstacles_lim = -detection_range_/2.0 + (0:1:nr_sectors) * delta_theta_obs;
            obj.obstacles_lim = obj.obstacles_lim';

            % sectors center
            theta_obs = -detection_range_/2.0 + delta_theta_obs/2.0 + (0:1:nr_sectors-1) * delta_theta_obs;
            theta_obs = round(theta_obs,6)';
            obj.theta_obs = theta_obs;
            obj.theta_obs_2pi = convert_pi_2pi(obj,theta_obs);

            obj.x_sensor_final = 30*cos(obj.theta_obs);
            obj.y_sensor_final = 30*sin(obj.theta_obs);
            obj.b_sensor_i = 0;

            %precompute transform matrices
            % R = rotx(alpha) * roty( beta ) * rotz(gamma);
            R = rotx(obj.m1_ori(1)) * roty( obj.m1_ori(2) ) * rotz(obj.m1_ori(3));
            obj.m01 = [ R obj.m1_pos'; 0 0 0 1];

            R = rotx(obj.m2_ori(1)) * roty( obj.m2_ori(2) ) * rotz(obj.m2_ori(3));
            obj.m02 = [ R obj.m2_pos'; 0 0 0 1];

            %R = rotx(obj.m3_ori(1)) * roty( obj.m3_ori(2) ) * rotz(obj.m3_ori(3));
            %obj.m03 = [ R obj.m3_pos'; 0 0 0 1];

            %R = rotx(obj.m4_ori(1)) * roty( obj.m4_ori(2) ) * rotz(obj.m4_ori(3));
            %obj.m04 = [ R obj.m4_pos'; 0 0 0 1];

            % compute distance from center to vehicle's margins
            % compute only for one side, the other can be mirrored
            psi = theta_obs( theta_obs < 0 );

            % 'a' stores the distances from center of rotation
            % perpendicular to robot margins
            % 'theta' are the angles between theta_obs and the same
            % perpendicular
            % 'd' is the effective distance from center of rotation to
            % robot margin, at the angle given by the theta_obs
            a = zeros(size(psi));
            theta = a;
            %d = a;

            % rear perpendicular distance
            a1 = obj.L/2 - obj.displc;
            % side perpendicular distance
            a2 = obj.W/2;

            % angle of the rear right vehicle corner
            alfa1 = atan(a2/a1);
            beta1 = - (pi - alfa1);

            % angle of the front right corner
            alfa2 = atan( (obj.L/2 + obj.displc ) / a2);
            beta2 = - (pi/2 - alfa2);

            % Indexs of the several branches
            cond_1_idx = psi < beta1;
            cond_2_idx = (beta1 <= psi) & (psi < beta2);
            cond_3_idx = psi >= beta2;

            % Fill up 'a'
            a(cond_1_idx) = obj.L/2 - obj.displc; %same as a1
            a(cond_2_idx) = obj.W/2;    %same as a2
            a(cond_3_idx) = obj.L/2 + obj.displc;

            % compute thetas
            theta(cond_1_idx) = pi - abs(psi(cond_1_idx));
            theta(cond_2_idx) = abs( abs(psi(cond_2_idx)) - pi/2 );
            theta(cond_3_idx) = abs( psi(cond_3_idx) );

            % finally 'd'
            d = a./cos(theta);

            % properly fill the array to use in future computations
            obj.in_veh_dist = zeros(size(obj.theta_obs));
            % right side


            aux = (obj.nr_sectors-1)/2;

            obj.in_veh_dist(1:aux) = d;

            % front center
            obj.in_veh_dist((obj.nr_sectors-1)/2 + 1) = obj.L/2 + obj.displc;

            % right side
            obj.in_veh_dist((obj.nr_sectors-1)/2 + 2 : end) = flipud(d);

            in_veh_dist = obj.in_veh_dist;

            obj.robot_distance = in_veh_dist;

            obj.min_sensor = 2;
            obj.max_sensor = nr_sectors-1;

        end

        function erro = trigger_obstacles(obj)
            erro = 0;
            [res , ~, obj.auxDataS1VS1, ~] = obj.vrep.simxReadVisionSensor(obj.clientID, obj.S1_VS1_handle, obj.vrep.simx_opmode_buffer);
            if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                msg = 'ERROR: Failed getting sensor data for S1_VS1! \n';
                error(msg);
                erro = 1;
            end
            [res , ~, obj.auxDataS1VS2, ~] = obj.vrep.simxReadVisionSensor(obj.clientID, obj.S1_VS2_handle, obj.vrep.simx_opmode_buffer);
            if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                msg = 'ERROR: Failed getting sensor data for S1_VS2! \n';
                error(msg);
                erro = 1;
            end
        end

        function [error,distances] = get_DistanceSensorAquisition(obj, show_obs, show_limits)
            error = 0;
            show_obstacles = false;
            show_sectors_limits = false;
            if nargin >= 2
                if show_obs ~= 0
                    show_obstacles = true;
                end
            end
            if nargin >= 3
                if show_limits ~= 0
                    show_sectors_limits = true;
                end
            end

            % Inf was returned when a sector was ocluded by the physical disposition of the laser scanners in the vehicle
            % and some obstacle did not allow the scanner to detect obstacles in other sectors.
            % NOTICE: with this configuration, there is no way to tell whether there is an obstacle or not. We can't know.

            % For now, a finit value will be returned. Nevertheless, behavior might change in future (returning again inf) if we
            % see that we must know that in such sectors it is impossible to tell whether there is an obstacle or not.

            distances = ones(obj.nr_sectors, 1) * obj.unknown_sector_method;
            % times 1.1 to ensure distance higher than the laser
            % scanner reach

            % need for error check: all sensors must be 1 pixel height
            % all sensors must have same width

            % size of incoming data, i.e. the number of steps provided by
            % sensors
            vs_size = obj.auxDataS1VS1(1);

            % get the obstacle position (relative to its own sensor)
            vect1 = reshape(obj.auxDataS1VS1(3:end), [4,vs_size]);
            vect2 = reshape(obj.auxDataS1VS2(3:end), [4,vs_size]);

            % positions [x y z 1]' for all obstacles (merge of sensors)
            pos = zeros(4, vs_size*2);


            for i = 1:vs_size
                pos( :, i)  = obj.m01 * [vect1(1:3, i);1];
                pos( :, i+vs_size)  = obj.m02 * [vect2(1:3, i);1];
            end

            % convert to polar coordinates
            [theta, rho] = cart2pol(pos(1,:), pos(2,:));

            % ensure values are within [-pi pi]
            theta = theta - 2*pi*floor( (theta+pi)/(2*pi) );


            final = floor(size(theta,2));

            IgnoredStepsInitS1 = size(obj.S1_VS1_ignore_steps,2);
            IgnotedStepsEndS1 = size(obj.S1_VS2_ignore_steps,2);


            % map each obstacle to the pre-define sectors
            for i = IgnoredStepsInitS1+2:final-IgnotedStepsEndS1
                if(theta(i-1)<theta(i))
                    angle_min = theta(i-1);
                    angle_max = theta(i);
                else
                    angle_min = theta(i);
                    angle_max = theta(i-1);
                end

                %resolução do problema quando angle_min próximo de -pi e
                %angle_max próximo de pi
                if angle_max-angle_min>pi
                    % o máximo e o mínimo alteram quando mudamos para 0 e
                    % 2pi
                    angle_min_2pi = convert_pi_2pi(obj,angle_max);
                    angle_max_2pi = convert_pi_2pi(obj,angle_min);
                    Index = find(obj.theta_obs_2pi<angle_max_2pi & obj.theta_obs_2pi>angle_min_2pi);
                else
                    Index = find(obj.theta_obs<angle_max & obj.theta_obs>angle_min);
                end

                if size(Index)>2
                    value = 1;
                end

                [lin,col]=size(Index);

                %for k=1:size(Index)
                for k=1:lin
                    j = Index(k);
                    if(pos(1,i)-pos(1,i-1)==0) %recta vertical do obstáculo x1-x0=0
                        m_sensor_i= (obj.y_sensor_final(j)-0)/(obj.x_sensor_final(j)-0);
                        b_recta_i = pos(1,i);

                        x_interception = b_recta_i;
                        y_interception = (m_sensor_i*x_interception)+obj.b_sensor_i;
                    elseif(obj.x_sensor_final(j)-0==0) %sensor da frente em que x1-x0=0
                        m_recta_i = (pos(2,i)-pos(2,i-1))/(pos(1,i)-pos(1,i-1));
                        b_recta_i = pos(2,i)-m_recta_i*pos(1,i);

                        x_interception = 0;
                        y_interception = (m_recta_i*x_interception)+b_recta_i;
                    else  %outros casos
                        m_sensor_i= (obj.y_sensor_final(j)-0)/(obj.x_sensor_final(j)-0);
                        m_recta_i = (pos(2,i)-pos(2,i-1))/(pos(1,i)-pos(1,i-1));
                        b_recta_i = pos(2,i)-m_recta_i*pos(1,i);

                        x_interception = (b_recta_i-obj.b_sensor_i)/(m_sensor_i-m_recta_i);
                        y_interception = (m_sensor_i*x_interception)+obj.b_sensor_i;

                    end
                    distance = sqrt((x_interception)^2+(y_interception)^2);

                    if distance < distances(j)
                        distances(j) = distance;
                    end
                end
            end

            if show_obstacles
                [X,Y] = pol2cart(obj.theta_obs, distances);
                [X1,Y1] = pol2cart(obj.theta_obs, obj.in_veh_dist);

                dat = [X,Y,X1,Y1,zeros(obj.nr_sectors,1)];
                dat = reshape(dat', 1, numel(dat));
                [res, ~, ~, ~, ~] = obj.vrep.simxCallScriptFunction(obj.clientID,'SICK_TIM310', 1, 'show_sectors_lines_function',[],dat',[],[],obj.vrep.simx_opmode_oneshot);
                if (res~=obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                    error('show_sectors_lines_function! \n');
                end
            end

            if show_sectors_limits
                [X1,Y1] = pol2cart(obj.obstacles_lim, ones(numel(obj.obstacles_lim),1)*30);

                dat = [X1,Y1,zeros(numel(obj.obstacles_lim),1)];
                dat = reshape(dat', 1, numel(dat));
                [res, ~, ~, ~, ~] = obj.vrep.simxCallScriptFunction(obj.clientID,'SICK_TIM310', 1, 'show_sectors_lim_lines_function',[],dat',[],[],obj.vrep.simx_opmode_oneshot);
                if (res~=obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                    error('show_sectors_lim_lines_function! \n');
                end
            end

            % return the distances from vehicle bounding box
            distances = (distances - obj.in_veh_dist)*100;      %cm
            % ensure distances are not smaller than vehicle itself
            distances( distances < 0 ) = 0;
        end

        % Gets robot dimension
        % W robot width
        % L robot lenght
        % H robot height
        function [error, W, L, H] = get_robot_dimension(obj)
            error = 0;
            W = obj.W * 100;    %m to cm
            L = obj.L * 100;    %m to cm
            H = obj.H * 100;    %m to cm
        end

         %Function that allows you to get the arm characteristics
    function [error,nJoints,Link,MinPositionJoint,MaxPositionJoint] = get_ManipulatorCharacteristics(obj)
        error = 0;
        nJoints = obj.number_of_joints;
        MinPositionJoint(1)=-360*pi/180;
        MinPositionJoint(2)=-90.00*pi/180;
        MinPositionJoint(3)=-131*pi/180;
        MinPositionJoint(4)=-102*pi/180;
        MinPositionJoint(5)=-90*pi/180;
        MaxPositionJoint(1)=360*pi/180;
        MaxPositionJoint(2)=75*pi/180;
        MaxPositionJoint(3)=131*pi/180;
        MaxPositionJoint(4)=102*pi/180;  
        MaxPositionJoint(5)=90*pi/180;  
        Link(1)=0.166;      % x distance from robot center
        Link(2)=0.103;       % z distance until joint 0
        Link(3)=0.033;       % x distance from joint 0 to joint 1
        Link(4)=0.147;        % z distance from joint 0 to joint 1
        Link(5)=0.155;       % z distance from joint 1 to joint 2
        Link(6)=0.135;      % z distance from joint 2 to joint 3
        Link(7)=0.097;        % z distance from joint 3 to joint 4
        Link(8)=0.1205;      % z distance from joint 4 to gripper
        obj.MinJointPos = MinPositionJoint;
        obj.MaxJointPos = MaxPositionJoint;
    end

    %Function that allows you to set the joints from the arm
        function [error] = set_joints(obj, armJoints) %armJoints (1-7) in rad
            error = 0;
            for i=1:1:obj.number_of_joints
                if armJoints(i)<obj.MinJointPos(i) || armJoints(i)>obj.MaxJointPos(i)
                    disp('ERROR: value outside the limits of the joints');
                    error=1;
                    return;
                else
                    res = obj.vrep.simxSetJointTargetPosition(obj.clientID, obj.jointHandle{i},armJoints(i), obj.vrep.simx_opmode_oneshot);

                    if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                        disp('ERROR: Failed sending Joint value!');
                        error=1;
                        return;
                    end
                end
            end
        end

        %Function that allows you to get the joints values from the arm
        %armJoints - joint values for arm (in rad)
        function [error,armJoints] = get_joints(obj)
            % Get pose
            % x and y in cm
            % phi in rad
            error = 0;
            for i = 1:obj.number_of_joints
                %Read from joints
                [res,armJoints(i)]=obj.vrep.simxGetJointPosition(obj.clientID, obj.jointHandle{i},obj.vrep.simx_opmode_buffer);
                if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                    disp('ERROR: Failed read Joint value!');
                    error = 1;
                    return;
                end
            end
        end

        %Functions that allows close the hand
        function error = close_hand(obj)       
            error = 0;
            res = obj.vrep.simxSetJointTargetPosition(obj.clientID, obj.HandHandle{1},0, obj.vrep.simx_opmode_oneshot);
            if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                disp('ERROR: Failed sending Joint value!');
                error=1;
                return;
            end

            res = obj.vrep.simxSetJointTargetPosition(obj.clientID, obj.HandHandle{2},0, obj.vrep.simx_opmode_oneshot);
            if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                disp('ERROR: Failed sending Joint value!');
                error=1;
                return;
            end
            
        end

        %Functions that allows open the hand
        function error = open_hand(obj)
            error = 0;
            res = obj.vrep.simxSetJointTargetPosition(obj.clientID, obj.HandHandle{1},0.025, obj.vrep.simx_opmode_oneshot);
            if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                disp('ERROR: Failed sending Joint value!');
                error=1;
                return;
            end

            res = obj.vrep.simxSetJointTargetPosition(obj.clientID, obj.HandHandle{2},-0.05, obj.vrep.simx_opmode_oneshot);
            if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                disp('ERROR: Failed sending Joint value!');
                error=1;
                return;
            end
        end

        function [output] = convert_pi_2pi(obj,input)
            output = zeros(size(input));
            for i=1:size(input)
                if input(i)<0
                    output(i) = input(i)+2*pi;
                else
                    output(i) = input(i);
                end
            end
        end

        function terminate(obj)
            %% Stop data streaming

            % Vision sensors for obstacles
            [res , ~, ~, ~] = obj.vrep.simxReadVisionSensor(obj.clientID, obj.S1_VS1_handle, obj.vrep.simx_opmode_discontinue);
            if (res ~= obj.vrep.simx_return_ok  && res ~= obj.vrep.simx_return_novalue_flag)
                msg = 'ERROR: Failed to stop data streaming for S1_VS1_handle';
                error(msg);
            end

            [res , ~, ~, ~] = obj.vrep.simxReadVisionSensor(obj.clientID, obj.S1_VS2_handle, obj.vrep.simx_opmode_discontinue);
            if (res ~= obj.vrep.simx_return_ok  && res ~= obj.vrep.simx_return_novalue_flag)
                msg = 'ERROR: Failed to stop data streaming for S1_VS2_handle';
                error(msg);
            end

            [res , ~, ~, ~] = obj.vrep.simxReadVisionSensor(obj.clientID, obj.S2_VS1_handle, obj.vrep.simx_opmode_discontinue);
            if (res ~= obj.vrep.simx_return_ok  && res ~= obj.vrep.simx_return_novalue_flag)
                msg = 'ERROR: Failed to stop data streaming for S2_VS1_handle';
                error(msg);
            end

            [res , ~, ~, ~] = obj.vrep.simxReadVisionSensor(obj.clientID, obj.S2_VS2_handle, obj.vrep.simx_opmode_discontinue);
            if (res ~= obj.vrep.simx_return_ok  && res ~= obj.vrep.simx_return_novalue_flag)
                msg = 'ERROR: Failed to stop data streaming for S2_VS2_handle';
                error(msg);
            end
        end
    end
end

