function calcolo_matrice_di_trasformazione

%VARIABILI DI GIUNTO: t1 = theta1, d2, t3 = theta3, t4 = theta4, t5 = theta5;
syms t1 d2 t3 t4 t5;
%d1 l3 d5;
syms d1 l3 d5;
T01 = [cos(t1) 0 -sin(t1) 0; sin(t1) 0 cos(t1) 0; 0 -1 0 d1; 0 0 0 1];
T12 = [1 0 0 0; 0 1 0 0; 0 0 1 d2; 0 0 0 1];
T23 = [cos(t3) -sin(t3) 0 l3*cos(t3); sin(t3) cos(t3) 0 l3*sin(t3); 0 0 1 0; 0 0 0 1];
T34 = [cos(t4) 0 -sin(t4) 0; sin(t4) 0 cos(t4) 0; 0 -1 0 0; 0 0 0 1];
T45 = [cos(t5) -sin(t5) 0 0; sin(t5) cos(t5) 0 0; 0 0 1 d5; 0 0 0 1];

T05 = T01*T12*T23*T34*T45