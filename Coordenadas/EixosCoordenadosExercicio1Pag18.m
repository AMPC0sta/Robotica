theta = deg2rad(30);

% Answer to 1.a)
disp('Exercice 1.a: Transformation Matrix from Coordinates System A to Coordinates System B');
AtB = [ 
        [1 0 0 0];
        [0 cos(theta) -sin(theta) 0];
        [0 sin(theta)  cos(theta) 0]; 
        [0 0 0 1] 
    ]
% Answer to 1.b)
ApOB = [0 0 0]';
ArB = [ 
        [1 0 0];
        [0 cos(theta) -sin(theta)];
        [0 sin(theta)  cos(theta)] 
    ];

ArBt = ArB';
BpOA = -ArBt * ApOB;

disp('Exercice 1.b: Transformation Inverted Matrix from Coordinates System B to Coordinates System A');
BtA =  [ArBt,BpOA;[0 0 0 1]]

Bp = [0.0 1.5 0.0]';

disp('Testing: Transforming point Bp into Ap, and into Bp again');
disp('Coordinates of point P from referential B to A');
Ap = BtA*[Bp;1]
disp('Coordinates of point P from referential A to B');
Calculated_Bp = AtB * Ap


    

    



