function [ POS ] = MCD( Q )

L0 = 0.11; L2 = 0.105; L3 = 0.133; 
L4 = 0.043; GARRA = 0.1;

q1 = Q(1); q2 = Q(2); q3 = Q(3); q4 = Q(4);

x = L2*cos(q1)*cos(q2) + L3*cos(q1)*cos(q2)*cos(q3) - L3*cos(q1)*sin(q2)*sin(q3) + cos(q2 + q3)*cos(q1)*cos(q4)*(GARRA + L4) - sin(q2 + q3)*cos(q1)*sin(q4)*(GARRA + L4);
y = L2*cos(q2)*sin(q1) + L3*cos(q2)*cos(q3)*sin(q1) - L3*sin(q1)*sin(q2)*sin(q3) + cos(q2 + q3)*cos(q4)*sin(q1)*(GARRA + L4) - sin(q2 + q3)*sin(q1)*sin(q4)*(GARRA + L4);
z = L0 + L3*sin(q2 + q3) + L2*sin(q2) + GARRA*sin(q2 + q3 + q4) + L4*sin(q2 + q3 + q4);
 

POS = [x;y;z];

end

