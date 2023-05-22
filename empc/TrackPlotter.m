figure;

plot3(0,0,0,'g.','MarkerSize',20);
hold on;
plot3( [ 0 1 1 0 0]',[ 0 0 1 1 0]' ,[ 1 1 1 1 1],'bo','MarkerSize',10,'LineWidth',3);
plot3( [0 0 1 1 0 0]',[0 0 0 1 1 0]' ,[0 1 1 1 1 1],'r-','LineWidth',3);
plot3(xHistory(:,1),xHistory(:,2),xHistory(:,3),'k.','MarkerSize',10);
xlabel('x position','FontSize',20);
ylabel('y position','FontSize',20);
zlabel('z position','FontSize',20);
set(gca,'FontSize',20)
axis([-0.5 1.5 -0.5 1.5 -0.5 1.5]);
grid on
title('Waypoint Following','FontSize',24)
legend('Origin','Waypoints','Track','Trajectory')