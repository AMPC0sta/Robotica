function trfs = transformations()
    % This wrapper function returns handles to the individual functions
    trfs.square = @from_world_to_kuka_matrix;
    trfs.square = @from_kuka_center_to_arm_matrix;
    
end

function Okuka = from_world_to_kuka_matrix(translation,phi)
    Okuka = [
                [cos(phi)   -sin(phi)   0       translation(1)];
                [sin(phi)   cos(phi)    0       translation(2)];
                [0          0           1       translation(3)];
                [0          0           0       1];
    ];
end

function OArm = from_kuka_center_to_arm_matrix(L)
    OArm = [
                [1  0   0   translation(1)];
                [0  1   0   translation(2)];
                [0  0   1   translation(3)];
                [0  0   0   1];
    ];
end
