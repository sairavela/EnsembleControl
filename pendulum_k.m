clear
clf

%define state
m_1 = 1; 
m_2 = 1; 
l_1 = 0.5; 
l_2 = 0.5;
total_t = 1.2;
t_wait = 0.1;
t_init = 0;
tspan = [t_init t_init+t_wait];
y0 = [pi/2 pi/2 0 0];
error = [0 0 0 0];     

moment_1 = 3/(m_1*l_1^2);
moment_2 = 3/(m_2*l_2^2);
y_forced =[0;0]; 
round = 1;

axis_l = l_1+l_2;
axis([-axis_l axis_l -axis_l axis_l])

%circle radius that can be clicked
center = [0, 0];
radius = l_1+l_2;
viscircles(center, radius);


x1 = [0 l_1*sin(y0(1))]; %x-coordinates for first pendulum
y1 = [0 -l_1*cos(y0(1))]; %y-coordinates for first pendulum
x2 = [l_1*sin(y0(1)) l_1*sin(y0(1))+l_2*sin(y0(2))];
y2 = [-l_1*cos(y0(1)) -l_1*cos(y0(1))-l_2*cos(y0(2))];
p1 = line(x1, y1);
p2 = line(x2, y2);

mv = 1;
num_ens = 100; %number of controllers to test

pos_ens = zeros(2, num_ens); % 'y' estimated position
t_ens = zeros(num_ens, 1);


%while loop should run without resetting goal until error (distance between
%goal and actual position) is minimized to value = 0.01
while(1)
    
    if ( rem(round, 150) == 1 || norm(mean(error, 2)) < 0.01)
        round = 1;
        %user-input/goal
        [x_g, y_g] = ginput(1);
        goal_pos = [x_g; y_g];
        %control
        mv = 1;
        % 'x' control input
    end
    %round = round + 1;
    
    %creating ensemble of controllers
    tic 
    %tstart = tic;
    u_ens = repmat(y_forced, 1, num_ens)+mv*randn(2, num_ens); 
    for index = (1:num_ens)
        u_forced = [u_ens(1,index);u_ens(2, index)];
        [~, y] = ode45(@(t, y) odefcn(y,l_1, l_2, m_1, m_2,u_forced), [0 0.05], y0);%position the pendulum would be with given control
        %y(:,1:2) = atan2(sin(y(:,1:2)),cos(y(:,1:2)));
        yend = y(end, :); %final position in theta
        x_pos = l_1*sin(yend(1))+l_2*sin(yend(2));
        y_pos = -l_1*cos(yend(1))-l_2*cos(yend(2));
%         dist = sqrt((x_pos-x_g).^2+(y_pos-y_g).^2);
%         t_i = find(min(dist));    
%         t_ens(i) = t_i; for changine time, try later
        pos_ens(:, index) = [x_pos, y_pos];
    end
    %t_for = toc(tstart)
    hold off
    v_control = zeros(2, num_ens);
    con_ens = [v_control;u_ens];
    %Remember, covariances from difference of var. from mean
    mean_pos = mean(pos_ens, 2);
    mu_pos_ens = pos_ens - mean_pos;
    
    error_ens = goal_pos - pos_ens + 0.05*randn(2,  num_ens); %E (error) = Z - y
    
    %singular value decomposition of mu_pos_ens
    [u_pos, s_pos, v_pos] = svd(mu_pos_ens);
    spr = rank(s_pos); 
    %only first 2 rows of s vector matters
    u_pos = u_pos(:, 1:spr);
    s_pos = s_pos(:, 1:spr);
    v_pos = v_pos(1:spr, 1:spr);
    
    con_ens = con_ens * (eye(num_ens) + mu_pos_ens' * u_pos * ...
        pinv( s_pos.^2 + max(.05^2, min(diag(s_pos)).^2)) * u_pos' * error_ens);
    
    u_ens = con_ens(3:4, :);
    mean_control = median(u_ens, 2);
    mv = diag(std(u_ens,[],2));
    
    
    %Plot
    hold on
    plot(x_g, y_g, 'rx');
    hold off
    plot(pos_ens(1,:), pos_ens(2,:), 'bx'); %predicted locations
    
    y_forced = mean_control;
    [t, y] = ode45(@(t, y) odefcn(y,l_1, l_2, m_1, m_2, y_forced),[0 0.0025], y0);
    num_points = length(y);
    
    t_1 = y(end, 1);
    t_2 = y(end, 2);
    
    theta1 = y(:, 1);
    theta2 = y(:, 2);

    x1 = l_1*sin(theta1); %x-coordinates for first pendulum
    y1 = -l_1*cos(theta1); %y-coordinates for first pendulum
    x2 = x1+l_2*sin(theta2);
    y2 = y1-l_2*cos(theta2);
        pts = [0 0; x1 y1; x2 y2;];
        
    
    
    
%     %track
%     hold on
%     plot(pos_ens(1,1),pos_ens(2,1), 'go')
%     plot(pos_ens(1,2),pos_ens(2,2), 'ro')
%     hold off
    
    
    for pt = 1:num_points
        delete(p1);
        delete(p2);
        
        hold on; 
        plot(0, 0, 'bx');
        axis([-axis_l axis_l -axis_l axis_l])
        plot(x_g, y_g, 'rx');
        hold off
        x_1 = [0 x1(pt)];
        y_1 = [0 y1(pt)];
        x_2 = [x1(pt) x2(pt)];
        y_2 = [y1(pt) y2(pt)];
        p1 = line(x_1, y_1);
        p2 = line(x_2, y_2);
        drawnow
    end
    
    y0 = y(end, :);
    x2_f = l_1*sin(y0(1))+l_2*sin(y0(2));
    y2_f = -l_1*cos(y0(1))-l_2*cos(y0(2));
    error = [x2_f, y2_f]' - goal_pos;
    
end