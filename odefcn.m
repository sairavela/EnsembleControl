 function dydt = odefcn(y, l_1, l_2, m_1, m_2, u)

error = [0.4521   -0.2882    4.1479   -2.2543];
dydt  = zeros(4, 1);
dydt(1) = y(3); %theta1 = y1 so theta1' = y3
dydt(2) = y(4); %theta2 = y2 so theta 2' = y4


g = 9.8;
c1 = l_1.^2*(4/3*m_1+m_2);
c2 = m_2*l_1*l_2;

c4 = m_2*l_2^2;
c5 = (m_1+m_2)*l_1;
moment = [c1 c2*cos(y(1)-y(2)); c2*cos(y(1)-y(2)) 4/3*c4];
force = [-c2*y(4)^2*sin(y(1)-y(2))-c5*g*sin(y(1)); c2*y(3)^2*sin(y(1)-y(2))-g*m_2*l_2*sin(y(3))];

sol = moment*force;
dydt(3) = sol(1)+u(1);
dydt(4) = sol(2)+u(2);


%% 
% error =
% 
%     0.4521   -0.2882    4.1479   -2.2543
% 

% 