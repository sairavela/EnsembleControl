clear
% Set function that will be solved to get solutions of positions
p_func=@doubleP;

% Set class parameters
horizon = 0.05;
cur_time = 0.0025;
mean_control = [0;0];
jj = 1; kk = 1;
nens = 20;

% Set parameters
l1 = 1.5;
l2 = 1.5;
m1 = 1;
m2 = 1;
params = [m1; l1; m2; l2];
mean_state = [pi/2*rand; pi/2*rand; 0; 0];
x0 =  [mean_state; mean_control; params];
state_ens = zeros(4,nens);

%Set Up Plot (and initial movment before control starts)
hold on; plot(0,0,'x');axis([-3 3 -3 3]);axis('square');hold off;
[t,x] = ode45(@doubleP,[0 cur_time],x0);
dt = t(2:end) - t(1:end-1);
dt = [dt; dt(end)];
for i = length(t):length(t)
    th1 = x(i,1)-pi/2; th2 = x(i,2)-pi/2;
    pts = [0 0; ...
        l1*cos(th1) l1*sin(th1);...
        l1*cos(th1)+l2*cos(th2) l1*sin(th1)+l2*sin(th2)];
    plot(0,0,'x');axis([-3 3 -3 3]);axis('square');
    line(pts(:,1), pts(:,2),'LineWidth',3);
        title('Ensemble Control','FontSize',14);
    drawnow;
    pause(dt(i));
end

mean_state = x(end, 1:4)';
x0 = [mean_state;mean_control;params];

while(1)
    if (rem(jj,200)==1 || norm(mean(goal_error,2))<0.05)
        jj=1;
        [xm,ym]=ginput(1);
        target = [xm; ym];
        %Create EnsControl instance for each goal
        p_sim = EnsControl;
        p_sim.target_ens = target;
        p_sim.horizon = horizon;
        p_sim.control_mean = mean_control;
        p_sim.state_fnc = p_func;
        p_sim.params = params;
    end
    jj = jj+1;
    
    p_sim.control_ens = p_sim.control_mean + randn(2,nens);
    state_ens = p_sim.Forecast(mean_state, nens, state_ens, p_sim.control_ens);
    p_sim.pred_ens = p_sim.ObsOperator(state_ens);
    p_sim.control_mean = p_sim.calc_update();
    
    % Plot each time step

    if(rem(jj,10) == 1)
        plot(target(1), target(2));
        hold on;
        plot(target(1),  target(2), 'ko');
        plot(p_sim.pred_ens(1,:), p_sim.pred_ens(2,:), 'bx');
        hold off;
    end
    
    th1 = mean_state(1)-pi/2; th2 = mean_state(2)-pi/2;
    pts = [0 0; ...
        l1*cos(th1) l1*sin(th1);...
        l1*cos(th1)+l2*cos(th2) l1*sin(th1)+l2*sin(th2)];

    hold on; plot(0,0,'x');axis([-3 3 -3 3]);axis('square');hold off;
    line(pts(:,1), pts(:,2),'LineWidth',3);
    title('Ensemble Control','FontSize',14);
    
    drawnow;
    
    x0 = [mean_state; p_sim.control_mean; params];
    [~,x] = ode45(@doubleP,[0 cur_time],x0);
    x(:,1:2) = atan2(sin(x(:,1:2)),cos(x(:,1:2)));          
    mean_state = x(end,1:4)';
    cur_loc = p_sim.ObsOperator(mean_state);
    goal_error = target - cur_loc;
    
end

