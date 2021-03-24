clear all;
l1 = 2;
l2 = 2;
m1 = 1;
m2 = 1;
params = [m1; l1; m2; l2];
mean_control = [0; 0];
mean_state = [pi/2; pi/2; 0; 0];
x0 =  [mean_state; mean_control; params];
[t,x] = ode45(@doubleP,0:0.1:10,x0);

%plot(t,x(:,1:2))
dt = t(2:end) - t(1:end-1);
dt = [dt; dt(end)];
figure(1);
for i = length(t):length(t)
    th1 = x(i,1)-pi/2; th2 = x(i,2)-pi/2;
    pts = [0 0; ...
        l1*cos(th1) l1*sin(th1);...
        l1*cos(th1)+l2*cos(th2) l1*sin(th1)+l2*sin(th2)];
    plot(0,0,'x');axis([-4 4 -4 4]);axis('square');
    line(pts(:,1), pts(:,2),'LineWidth',2);
    drawnow;
    pause(dt(i));
end
xlist =[ -2 0; -1.2 1.1;  -1.5 -1; -1 -2];

%%
performance_var = 1e-4;
mean_state = x(end,1:4)';
mean_control = [0; 0];
jj = 1;kk=1;
endt = 0.025;
mv = 8/endt;
nens = 10;
state_ens = zeros(4,nens);
idxx=1;
while(kk<=size(xlist,1)+1)
    control_ens = mean_control+randn(2,nens)*mv/2;
    ic = [repmat(mean_state,[1 nens]); control_ens];
    for i = 1:nens
        x0 = [ic(:,i); params];
        [~,x] = ode45(@doubleP,[0 endt],x0);
        x(:,1:2) = atan2(sin(x(:,1:2)),cos(x(:,1:2)));
        state_ens(:,i) = x(end,1:4)';
    end
    
    pos_ens = ObservationEq(state_ens, params);
    mean_pos = mean(pos_ens,2); dev_pos = pos_ens - mean_pos;
    [up,sp,vp] = svd(dev_pos);
    spr = rank(sp); up=up(:,1:spr); vp = vp(:,1:spr); sp = sp(1:spr,1:spr);
    
    if (rem(jj,100)==1)
        jj=1;
        [xm] = xlist(min(kk,size(xlist,1)),1); ym=xlist(min(kk,size(xlist,1)),2);
        kk=kk+1;
        ms = linspace(0,1,10);
        goal_points = [xm;ym].*ms + (1-ms).*mean_pos;
    end
    jj=jj+1;
    goal_pos = [xm;ym];%goal_points(:,floor(jj/10)+1);
       
    target_error_ens = goal_pos - pos_ens;
    
    kin_state = state_ens(1:2,:); 
    mean_kin  = mean(kin_state,2); dev_kin = kin_state - mean_kin;
    
    est_kin= kin_state*(eye(nens)+dev_pos'*up'*pinv(sp.*sp+0.0001)*up'*target_error_ens);
    est_kin_mean = mean(est_kin,2);
    des_state = [atan2(sin(est_kin),cos(est_kin))];
    des_pos = ObservationEq(des_state, params);
    qual = sum(abs(des_pos - goal_pos));
    idx = find(qual==min(qual));
    des_pos = des_pos(:,idx(1)); des_state = [des_state(:,idx(1)); 0; 0];
    dev_state = state_ens - mean(state_ens,2);
    [u,s,v] = svd(dev_state);
    sr = rank(s); u=u(:,1:sr);    s = s(1:sr,1:sr); v = v(:,1:sr);
    
    mean_control = mean(control_ens,2);
    dev_control = control_ens - mean_control;
    
    a = atan2(sin(des_state(1:2)).*cos(state_ens(1:2,:))-...
          cos(des_state(1:2)).*sin(state_ens(1:2,:)),...
          cos(des_state(1:2)).*cos(state_ens(1:2,:))+...
          sin(des_state(1:2)).*sin(state_ens(1:2,:)));
      
      b=    des_state(3:end) -state_ens(3:end,:);
    state_error_ens = [a;b];
    disp([rad2deg(des_state(1:2,:)) rad2deg(state_ens(1:2,:))]);
   control_ens = control_ens+dev_control*...
    dev_state'*u*pinv(s.*s+performance_var*eye(sr))*u'*state_error_ens;

    mean_control = mean(control_ens,2);
    %mv = min(svds(control_ens,1),1500);
    disp([ mean_pos goal_pos]');
    plot(goal_pos(1),goal_pos(2));
    hold on
    plot(pos_ens(1,:),pos_ens(2,:),'x');
    plot(des_pos(1,:),des_pos(2,:),'g+');
    plot(goal_pos(1),goal_pos(2),'ko');
    hold off;
    
    
    th1 = mean_state(1)-pi/2; th2 = mean_state(2)-pi/2;
    pts = [0 0; ...
        l1*cos(th1) l1*sin(th1);...
        l1*cos(th1)+l2*cos(th2) l1*sin(th1)+l2*sin(th2)];
    hold on; plot(0,0,'x');axis([-4 4 -4 4]);axis('square');hold off;
    line(pts(:,1), pts(:,2),'LineWidth',2);
    title('Ensemble Control','FontSize',14);
    
    drawnow;
    frame = getframe(gcf);
    im{idxx} = frame2im(frame);
    idxx = idxx+1
      x0 = [mean_state; mean_control; params];
    [t,x] = ode45(@doubleP,[0 0.01],x0);
    x(:,1:2) = atan2(sin(x(:,1:2)),cos(x(:,1:2)));
            
    mean_state = x(end,1:4)';
end


%%
filename = 'testAnimated.gif'; % Specify the output file name
for i = 1:5:idxx-1
    [A,map] = rgb2ind(im{i},256);
    if i == 1
        imwrite(A,map,filename,'gif','LoopCount',Inf,'DelayTime',00);
    else
        imwrite(A,map,filename,'gif','WriteMode','append','DelayTime',0.0);
    end
end

