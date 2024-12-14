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
disp(ArB);
