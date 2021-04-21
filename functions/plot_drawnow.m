figure(123)
clf;
title(strcat('Time :' , num2str(time)))
hold on
axis equal
grid on
set(gcf,'color','white'); 
pos = x(1); theta = x(3) - pi/2; 
LW = 2;
J = customcolormap_preset('pasteljet');
plot([pos-box_length/1, pos+box_length/1, pos+box_length/1, pos-box_length/1, pos-box_length/1],...
     [0, 0, box_length, box_length, 0],'LineWidth',LW,'Color','k')
plot([pos pos+L*cos(theta)],[box_length/2, box_length/2+L*sin(theta)],'LineWidth',LW*2,'Color',J(1,:))
filledCircle([pos+L*cos(theta), box_length/2+L*sin(theta)],radius,100,J(200,:));
filledCircle(obs(1:2),obs(3),100,'k');
xlimit = 30;
plot([-xlimit, xlimit], [-0.2, -0.2],'k','LineWidth',1)
xlim([-xlimit xlimit])
ylim([-L L*2])
drawnow


function h = filledCircle(center,r,N,c)
%---------------------------------------------------------------------------------------------
% FILLEDCIRCLE Filled circle drawing
% 
% filledCircle(CENTER,R,N,COLOR) draws a circle filled with COLOR that 
% has CENTER as its center and R as its radius, by using N points on the 
% periphery.
%
% Usage Examples,
%
% filledCircle([1,3],3,1000,'b'); 
% filledCircle([2,4],2,1000,'r');
%
% Sadik Hava <sadik.hava@gmail.com>
% May, 2010
%
% Inspired by: circle.m [Author: Zhenhai Wang]
%---------------------------------------------------------------------------------------------
THETA=linspace(0,2*pi,N);
RHO=ones(1,N)*r;
[X,Y] = pol2cart(THETA,RHO);
X=X+center(1);
Y=Y+center(2);
h=fill(X,Y,c);
end