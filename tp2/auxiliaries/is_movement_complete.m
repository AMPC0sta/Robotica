function R = is_movement_complete(set_joints, read_joints, tolerance)
% function type: Auxiliary
% 
% Objective: Determine if arm already moved its joints to target angles
% 
% Input:
%       set_joints      - Array of target angles for arm joints
%       read_joints     - Array with status of arm joints
%       tolerance       - Allowed slack between target angles and actual angles (real world robots has slacks)
% 
% Output
%       0               - Alias OK, if difference between target and actual joints are smaller than tolerance
%       1               - Alias NOK, if end-efector didn't reached the target yet.

    OK = 0;
    NOK = 1;

    acc = 0;
    R = NOK;  % default output value

    % for each joint in the array
    for ptr = 1:length(set_joints)
        if abs(set_joints(ptr) - read_joints(ptr)) >= tolerance
            acc = acc + 1;               % in not met, than increase counter
        end
    end

    % if counter wasnt increased, all joints are under the tolerance.
    if acc == 0
        R = OK;
    end
end