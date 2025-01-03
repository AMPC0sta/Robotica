function  [function_error,R] = kuka_inverse_kinematics_2dof(pe,L,S,qmin,qmax)

    function_error = 0;


    % intermediary/auxiliary values
    xed = pe(1);
    zed = pe(2);

    alfa1 = atan2(zed,xed);
    r = sqrt(xed^2 + zed^2);
    arg1 = (r^2 + L(1)^2 - L(2)^2) / (2*L(1) * r);                          % argument to compute theta1

    arg2 = ( L(1)^2 +L(2)^2 - r^2) / (2*L(1) * L(2) );                               % argument to compute theta2
    disp(arg1);
    
    % is this a valid solution?
    if(arg1 >= -1 && arg1 <= 1)
        beta1 = cos(arg1);
    else
        function_error = 1; %#ok<*NASGU>
        error("There's no solution for Joint1/Theta1!");
    end

    if(arg2 >= -1 && arg2 <=1 )
        beta2 = acos(arg2);
    else
        function_error = 1;
        error("There's no solution for Joint2/Theta2!");
    end

    % desired arm elbow position
    if S==1                                                         % elbow right
        theta_1 = pi/2 - (alfa1-beta1);
        theta_2 = pi - beta2;
    elseif S==-1                                                    % elbow left
        theta_1 = pi/2 - (alfa1+beta1);
        theta_2 = -(pi-beta2);
    elseif S ~= -1 && S ~= 1
        function_error = 1;
        error("No recognizable value for argument function S");
    end

    % checking solutions with joints boundaries
    if (theta_1 <= qmin(1) || theta_1 > qmax(1))
        function_error = 1;
        error("Theta1 is outside of Joint01 limits!");
    end

    if (theta_2 <= qmin(2) || theta_2 > qmax(2))
        function_error = 1;
        error("Theta2 is outside of Joint02 limits!");
    end

    R = [theta_1,theta_2]';

end

