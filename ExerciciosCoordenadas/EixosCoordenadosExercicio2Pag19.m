
% First  Operation: Rotation of 30 degrees in Z
% Second Operation: Translation along the vector (4,3,0)
theta = deg2rad(30);  % Rotation in Z
trans_X = 4;
trans_Y = 3;
trans_Z = 0;

% anonymous functions, can be called inside this very script
% for rotation
RotZ = @(x) [ 
                [1 0 0 0];
                [0 cos(x) -sin(x) 0];
                [0 sin(x)  cos(x) 0]; 
                [0 0 0 1] 
            ];

% for translation
Trans = @(x,y,z) [
                    [1 0 0 x];
                    [0 1 0 y];
                    [0 0 1 z];
                    [0 0 0 1];
                ];



disp("Exercice 2a: Transformation Matrix from A to B");
AtB = Trans(trans_X,trans_Y,trans_Z) * RotZ(theta);
disp(AtB);


disp("Exercice 2b: Point P1(3,7,0) from Referential B to A (implies inverse calcualtion)");
Bp1 = [3.0 7.0 0.0]';

ArB = AtB(1:3,1:3);                     % get the non homegeneous matrix 
ArBt = ArB';                            % transpose it
ApOB = AtB(1:3,4:4);                    % get the origin vector/point on A referential
BpOA = - ArBt * ApOB;                   % origin vector/point on B referential

BtA = [ArBt,BpOA;[0 0 0 1]];
Ap1 = AtB * [Bp1;1];
disp(Ap1);

disp("Exercice 2b: Checking if transformations matrices are injective");
New_Bp1 = BtA * Ap1;
disp(New_Bp1);

disp("Exercice 2c: Inverse matrix of BtA (resulted matrix must match 2a result)");
BrA = BtA(1:3,1:3);
BrAt = BrA';
New_BpOA = BtA(1:3,4:4);
New_ApOB = - BrAt * New_BpOA;
New_AtB = [BrAt,New_ApOB;[0 0 0 1]];
disp(New_AtB);

disp("Exercice 2d: Ordering transformation operations in a different way will result in a different matrix");
AtB_reordered =  RotZ(theta) * Trans(trans_X,trans_Y,trans_Z);
disp(AtB_reordered);