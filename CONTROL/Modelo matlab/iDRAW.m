%% RESETEAMOS LA MEMORIA
close all
clear all
clc

%% INCLUIMOS EL TOOLBOX
startup_rvc;


%% DESACOPLAMOS EL BRAZO PARA CONTROLARLO Y NOS QUEDAMOS CON EL BRAZO 3-DOF
L0 = 0.11; L1 = 0; L2 = 0.105; L3 = 0.133; 
L4 = 0.05; GARRA = 0.127;

L(1) = Link([0 0 0 pi/2 0]);
L(2) = Link([0 0 L2 0 0]);
L(3) = Link([0 0 L3 0 0]);
L(4) = Link([0 0 GARRA+L4 0 0]); % MUÑECA 1 (INCLINAR)

MiRobot = SerialLink(L, 'name', 'iDRAW');
MiRobot.base = transl(0, 0, L0);
MiRobot.plot([0 0 0 0]);

return

%% CALCULAMOS EXPLÍCITAMENTE LAS MATRICES HOMOGÉNEAS
syms q1 q2 q3 q4 L0 L2 L3 L4 GARRA
PI = sym('pi');
T1 = MDH(q1, L0, 0, PI/2);
T2 = MDH(q2, 0, L2, 0);
T3 = MDH(q3, 0, L3, 0);
T4 = MDH(q4, 0, L4+GARRA, 0);
Tt = simplify(T1*T2*T3*T4);

MCD1 = simplify([Tt(1,4); Tt(2,4); Tt(3,4)])

return

