function R = is_movement_complete(set_joints, read_joints, tolerance)

    error = 0;

    OK = 0;
    NOK = 1;

    acc = 0;

    R = NOK;
    for ptr = 1:length(set_joints)
        if abs(set_joints(ptr) - read_joints(ptr)) >= tolerance
            acc = acc + 1;
        else
            wait = 0;
        end
    end

        if acc == 0
            R = OK;
        end
end