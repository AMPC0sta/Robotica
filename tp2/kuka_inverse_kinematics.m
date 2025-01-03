function [function_error,R] = kuka_inverse_kinematics(pe,L,S,qmin,qmax,alfa)
    
    xed = pe(1);
    zed = pe(2);
    

    xi = xed - L(3) * cos(alfa);
    zi = zed - L(3) * sin(alfa);
    disp(xi);
    disp(zi);
    [function_error,out] = kuka_inverse_kinematics_2dof([xi,zi],L,S,qmin,qmax);

    theta_1 = out(1);
    theta_2 = out(2);

    theta_3 = alfa - ((pi/2 - theta_1) - theta_2);
    
    if( theta_3 <= qmin(3) || theta_3 >= qmax(3))
        function_error = 1; %#ok<*NASGU>
        error("Theta3/Joint03 is outside of joint limits");
    end

    R = [theta_1,theta_2,theta_3];