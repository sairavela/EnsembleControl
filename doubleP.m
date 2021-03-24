function xdot = doubleP(t,x)
g = 9.81;
lx = size(x);
params = x(7:end);
u = x(5:6);
x = x(1:4);
m1 = params(1); l1 = params(2); m2 = params(3); l2 = params(4);

th1 = rem(x(1),2*pi); th1dot = x(3); 
th2 = rem(x(2),2*pi); th2dot = x(4);

LHS = [(m1+m2)*l1               m2*l2*cos(th2-th1);...
        l1*cos(th2-th1)         l2                 ];

RHS = [m2*l2*th2dot^2*sin(th2-th1) - (m1+m2)*g*sin(th1);...
       -l1*th1dot^2*sin(th2-th1) - g*sin(th2)];
   
acc =  inv(LHS)*RHS;
xdot = zeros(lx);
xdot(1) = x(3);
xdot(2) = x(4);
xdot(3) = acc(1)+u(1);
xdot(4) = acc(2)+u(2);
xdot = xdot(:);
end