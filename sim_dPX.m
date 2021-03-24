clear all;
l1 = 2;
l2 = 2;
m1 = 1;
m2 = 1;
params = [m1; l1; m2; l2];
mean_control = [0; 0];
mean_state = [pi/2*rand; pi/2*rand; 0; 0];
x0 =  [mean_state; mean_control; params];
[t,x] = ode45(@doubleP,0:0.1:10,x0);

%plot(t,x(:,1:2))
dt = t(2:end) - t(1:end-1);
dt = [dt; dt(end)];
figure(1);set(gcf,'color','w');
for i = length(t):length(t)
    th1 = x(i,1)-pi/2; th2 = x(i,2)-pi/2;
    pts = [0 0; ...
        l1*cos(th1) l1*sin(th1);...
        l1*cos(th1)+l2*cos(th2) l1*sin(th1)+l2*sin(th2)];
    plot(0,0,'x');axis([-4 4 -4 4]);axis('square');
    line(pts(:,1), pts(:,2),'LineWidth',3);
        title('Ensemble Control','FontSize',14);
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
mv = 1;
nens = 20;
state_ens = zeros(4,nens);
idxx=1;
while(1)
     if (rem(jj,200)==1 || norm(mean(goal_error,2))<0.05)
        jj=1;
                 soundsc(sin((1:1024).^2*0.00025)/1024/1024,8192)

        [xm,ym]=ginput(1);
         goal_pos = [xm;ym];
         mv = 1;
             plot(goal_pos(1),goal_pos(2),'ko');
         goal_error = 1000;
    end
    jj=jj+1;
    control_ens = mean_control+mv*randn(2,nens);
    ic = [repmat(mean_state,[1 nens]); control_ens];
    for i = 1:nens
        x0 = [ic(:,i); params];
        [~,x] = ode45(@doubleP,[0 0.05],x0);
        x(:,1:2) = atan2(sin(x(:,1:2)),cos(x(:,1:2)));
        state_ens(:,i) = x(end,1:4)';
%         hold on;
%         for tt = 1:size(x,1)
%             pp = ObservationEq(x(tt,1:4)',params);
%         plot(pp(1),pp(2),'+');
%         end
    end
    hold off;
    pos_ens = ObservationEq(state_ens, params);
    mean_pos = mean(pos_ens,2); dev_pos = pos_ens - mean_pos;
    
    [up,sp,vp] = svd(dev_pos);
    spr = rank(sp); 
    up=up(:,1:spr); vp = vp(:,1:spr); sp = sp(1:spr,1:spr);
    
   
    target_error_ens = goal_pos+randn(2,nens)*0.05 - pos_ens;

    control_ens  = control_ens*...
     (eye(nens)+dev_pos'*...
      up*pinv(sp.*sp+max(0.0025,min(diag(sp)).^2))*up'*...
      target_error_ens);
    mean_control = median(control_ens,2);
   mv = diag(std(control_ens,[],2));
    if (rem(jj,10)==1)
                plot(goal_pos(1),goal_pos(2));
    hold on
    plot(pos_ens(1,:),pos_ens(2,:),'bx');
    plot(goal_pos(1),goal_pos(2),'ko');
    hold off;
   end
    th1 = mean_state(1)-pi/2; th2 = mean_state(2)-pi/2;
    pts = [0 0; ...
        l1*cos(th1) l1*sin(th1);...
        l1*cos(th1)+l2*cos(th2) l1*sin(th1)+l2*sin(th2)];
    hold on; plot(0,0,'x');axis([-4 4 -4 4]);axis('square');hold off;
    line(pts(:,1), pts(:,2),'LineWidth',3);
    title('Ensemble Control','FontSize',14);
    
    drawnow;
   % frame = getframe(gcf);
   % im{idxx} = frame2im(frame);
   % idxx = idxx+1
      x0 = [mean_state; mean_control; params];
    [t,x] = ode45(@doubleP,[0 0.0025],x0);
    x(:,1:2) = atan2(sin(x(:,1:2)),cos(x(:,1:2)));          
    mean_state = x(end,1:4)';
    whereami = ObservationEq(mean_state,params);
    goal_error = goal_pos - whereami;
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

