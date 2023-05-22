% This script plots the closed-loop responses of the nonlinear MPC
% controller used in the quadrotor path following example.

% Copyright 2019 The MathWorks, Inc.

% Plot the closed-loop response.
time = 0:stepTs:Duration;
lt = length(time);
lw=4; lw2 = 2;
pmin=-1.1;pmax = 1.1;
amin = -pi-pi/10; amax = pi+pi/10;
% Plot the states.
figure('Name','States')

title('Quadcopter Position','FontSize',24);
subplot(2,3,1)
hold on
plot(time,xHistory(:,1),'LineWidth',lw)
plot(time,repmat(xg(1),[1 lt]),'LineWidth',lw,'LineStyle','--')
grid on
xlabel('time','FontSize',24);
ylabel('x','FontSize',24);
legend('actual','reference','Location','southeast','FontSize',20)
title('QC X position','FontSize',28);
axis([0 Duration pmin pmax]);
set(gca,'FontSize',20);
subplot(2,3,2)
hold on
plot(time,xHistory(:,2),'LineWidth',lw)
plot(time,repmat(xg(2),[1 lt]),'LineWidth',lw,'LineStyle','--')
grid on
xlabel('time','FontSize',24);
ylabel('y','FontSize',24);
legend('actual','reference','Location','southeast','FontSize',20)
title('QC Y position','FontSize',28);
axis([0 Duration pmin pmax]);
set(gca,'FontSize',20);
subplot(2,3,3)
hold on
plot(time,xHistory(:,3),'LineWidth',lw)
plot(time,repmat(xg(3),[1 lt]),'LineWidth',lw,'LineStyle','--')
grid on
xlabel('time','FontSize',24);
ylabel('z','FontSize',24);
legend('actual','reference','Location','southeast','FontSize',20)
title('QC Z position','FontSize',28);
axis([0 Duration pmin pmax]);
set(gca,'FontSize',20);
subplot(2,3,4)
hold on
plot(time,xHistory(:,4),'LineWidth',lw)
plot(time,repmat(xg(4),[1 lt]),'LineWidth',lw,'LineStyle','--')
grid on
xlabel('time','FontSize',24);
ylabel('\phi','FontSize',24);
legend('actual','reference','Location','southeast','FontSize',20)
title('QC \phi angle','FontSize',28);
axis([0 Duration -pi-pi/10 pi+pi/10 ]);
set(gca,'FontSize',20);
subplot(2,3,5)
hold on
plot(time,xHistory(:,5),'LineWidth',lw)
plot(time,repmat(xg(5),[1 lt]),'LineWidth',lw,'LineStyle','--')
grid on
xlabel('time','FontSize',24);
ylabel('\theta','FontSize',24);
legend('actual','reference','Location','southeast','FontSize',20)
title('QC \theta angle','FontSize',28);
axis([0 Duration -pi-pi/10 pi+pi/10 ]);
set(gca,'FontSize',20);
subplot(2,3,6)
hold on
plot(time,xHistory(:,6),'LineWidth',lw)
plot(time,repmat(xg(6),[1 lt]),'LineWidth',lw,'LineStyle','--')
grid on
xlabel('time','FontSize',24);
ylabel('\psi','FontSize',24);
legend('actual','reference','Location','southeast','FontSize',20)
title('QC \psi angle','FontSize',28);
axis([0 Duration -pi-pi/10 pi+pi/10 ]);
set(gca,'FontSize',20);
% Plot the manipulated variables.
figure('Name','Control Inputs')

subplot(2,2,1)
hold on
stairs(time,uHistory(:,1),'LineWidth',lw)
ylim([-0.5,7.5])

plot(time,nloptions.MVTarget(2)*ones(1,length(time)),'LineWidth',lw,'LineStyle','--')
grid on
xlabel('time','FontSize',24);
ylabel('Actuation','FontSize',24);
legend('actual','hover')
title('Input 1','FontSize',28);
set(gca,'FontSize',20);

subplot(2,2,2)
hold on
stairs(time,uHistory(:,2),'LineWidth',lw)
ylim([-0.5,7.5])
plot(time,nloptions.MVTarget(2)*ones(1,length(time)),'LineWidth',lw,'LineStyle','--')
grid on
xlabel('time','FontSize',24);
ylabel('Actuation','FontSize',24);
title('Input 2','FontSize',28);
legend('actual','hover','FontSize',20)
set(gca,'FontSize',20);

subplot(2,2,3)
hold on
stairs(time,uHistory(:,3),'LineWidth',lw)
ylim([-0.5,7.5])
plot(time,nloptions.MVTarget(2)*ones(1,length(time)),'LineWidth',lw,'LineStyle','--')
grid on
xlabel('time','FontSize',24);
ylabel('Actuation','FontSize',24);
title('Input 3','FontSize',28);
legend('actual','hover','FontSize',20)
set(gca,'FontSize',20);
subplot(2,2,4)
hold on
stairs(time,uHistory(:,4),'LineWidth',lw)
ylim([-0.5,7.5])
plot(time,nloptions.MVTarget(2)*ones(1,length(time)),'LineWidth',lw,'LineStyle','--')
grid on
xlabel('time','FontSize',24);
ylabel('Actuation','FontSize',24);
title('Input 4','FontSize',28);
legend('actual','hover','FontSize',20)
set(gca,'FontSize',20);

