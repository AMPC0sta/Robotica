% Exercicio 1) 
% Resolvido em EixosCoordenadosExercicio1Pag18.m

% Exercicio 2.a)
% Resolvido em EixosCoordenadosExercicio2Pag19.m

% Exercicio 2.b)
% Resolvido em EixosCoordenadosExercicio2Pag19.m

% Exercicio 2.c)
% Resolvido em EixosCoordenadosExercicio2Pag19.m

% Exercicio 2.d

% Ponto P2)
Ap2 = [2 4 0]';

% Copiando os artefactos interessantes de EixosCoordenadosExercicio2Pag19.m

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

AtB = Trans(trans_X,trans_Y,trans_Z) * RotZ(theta);

Bp2 = AtB * [Ap2;1];

disp("Exercice 2d: Point P2 in referential B");
disp(Bp2);

% Exercicio 3) resolvido em EixosCoordenadosExercicio2Pag19.m (2.d)

% Exercicio 4) 

% Dois sistemas de coordenadas {A} e {B} coincidentes/sobrepostos.
% Operações sobre {B}:
%      1ª RotY(theta)  -> sobre Ya ou Yb
%      2ª RotZ(phi)    -Z sobre o novo Zb

% Porque as operções, não são relativas ao referencial base ( a sequencia de transformaçoes tera uma ordem diferente.)

% ArB = RotY(theta) * RotZ(phi)

% anonymous functions, can be called inside this very script
% for rotation
RotZ = @(x) [ 
                [1 0 0 0];
                [0 cos(x) -sin(x) 0];
                [0 sin(x)  cos(x) 0]; 
                [0 0 0 1] 
            ];

RotY = @(x) [
               [ cos(x) 0 sin(x) 0];
               [ 0 1 0 0];
               [-sin(x) 0 cos(x) 0];
               [ 0 0 0 1]
            ];

% since there's no assigned value for angles theta and  phi they will worked as symbols.
syms theta phi;

% multiplying matrices in post order multiplication. 
ArB = RotY(theta) * RotZ(phi);
disp('Exercice 4): resulted transformation matrix')
disp(ArB(1:3,1:3));


% Exercicio 5) 
% Considering the homogeneous matrix

syms thta;      % to be worked as symbolic

funcBtA = @(thta)  [
                    [1  0           0           1];
                    [0  cos(thta)   -sin(thta)  2];
                    [0  sin(thta)   cos(thta)   3];
                    [0  0           0           1]
               ];

disp('Exercice 5a) Resultant transformation matrix');
BtA = funcBtA(thta);
BrA = BtA(1:3,1:3);             % Non homogeneouns transformation matrix
BrAt = BrA';                    % Transposing it
BoAP = BtA(1:3,4:4);
AoBP = - BrAt * BoAP;
AtB = [BrAt,AoBP;[0 0 0 1]];
disp(AtB);

disp('Exercice 5b) Resultant transformation matrix, if theta 45 and Ap = [4 5 6]');

thta = deg2rad(45);
Bp = [4 5 6]';

BtA = funcBtA(deg2rad(thta));
BrA = BtA(1:3,1:3);             % Non homogeneouns transformation matrix
BrAt = BrA';                    % Transposing it
BoAP = BtA(1:3,4:4);
AoBP = - BrAt * BoAP;
AtB = [BrAt,AoBP;[0 0 0 1]];

Ap = AtB * [Bp;1];
disp(Ap);
disp('Checking if it goes backward to the original point');
New_Bp = BtA * Ap;
disp(New_Bp);


% Exercicio 6) 
% Considering the homogeneous matrices

disp('Exercice 6a) not solved in here');
disp();
disp('Exercice 6b) not solved in here');

AtB =   [
            [0.866  -0.5    0       11];
            [0.5    0.866   0       -1];
            [0      0       1       8];
            [0      0       0       1]
        ];

CtB =   [
            [1      0       0       11];
            [0      0.866   -0.5    10];
            [0      0.5     0.866   -20];
            [0      0       0       1]
        ];

DtA =   [
            [0.866  -0.5    0       -3];
            [0.433  0.75    -0.5    -3];
            [0.25   0.433   0.866   3];
            [0      0       0       1]
        ];
