function [ Q ] = MCI( POS )

L0 = 0.11; L2 = 0.105; L3 = 0.133; 
L4 = 0.043; GARRA = 0.1;

x1 = POS(1); y1 = POS(2); z1 = POS(3)-L0;
modu = sqrt(x1^2 + y1^2);  u = [x1/modu y1/modu]; % Nos situamos en el punto de conexión de la garra
u = (modu-GARRA-L4)*u;
x = u(1); y = u(2); z = z1;
r = sqrt(x^2+y^2);
d = sqrt(r^2+z^2);
alpha = atan(z/r);
beta = atan(sqrt(1-((d^2+L2^2-L3^2)/(2*d*L2))^2) /((d^2+L2^2-L3^2)/(2*d*L2)));
% gamma = atan(sqrt(1-((L3^2+L2^2-d^2)/(2*L3*L2))^2) / ((L3^2+L2^2-d^2)/(2*L3*L2)))
gamma = acos((L3^2+L2^2-d^2)/(2*L3*L2));

q2 = alpha + beta;
q3 = gamma - pi;
q4 = - q2 - q3;
q1 = atan(y/x);

Q = [q1; q2; q3; q4]*180/pi;

end