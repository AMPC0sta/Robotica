function trfs = transformations()
    % This wrapper function returns handles to the individual functions
    trfs.from_world_to_kuka_matrix = @from_world_to_kuka_matrix;
    trfs.from_kuka_center_to_arm_matrix = @from_kuka_center_to_arm_matrix;
    trfs.from_world_to_arm_base_matrix = @from_world_to_arm_base_matrix;
    trfs.invert_transformation_matrix = @invert_transformation_matrix;
    
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
                [1  0   0   L];
                [0  1   0   0];
                [0  0   1   0];
                [0  0   0   1];
    ];
end

function OWArm = from_world_to_arm_base_matrix(translation,phi,L)
    OWArm = from_world_to_kuka_matrix(translation,phi) * from_kuka_center_to_arm_matrix(L);
end


function Inv = invert_transformation_matrix(AmB)
    BrA = AmB(1:3,1:3)';
    AoP = AmB(1:3,4:4);
    BoP = -BrA * AoP;
    BmA = [BrA, BoP;[0 0 0 1]];

    Inv = BmA;
end

