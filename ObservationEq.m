function pos = ObservationEq(state, params)

L1 = params(2); L2 = params(4);
th1 = state(1,:)-pi/2; th2 = state(2,:)-pi/2;

pos = [L1*cos(th1)+L2*cos(th2); L1*sin(th1)+L2*sin(th2)];
end