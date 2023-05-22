function satyes = GoalSat(x,y)

if mean(abs(x-y))< 0.001 
    satyes = 1;
else
    satyes = 0;
end
    