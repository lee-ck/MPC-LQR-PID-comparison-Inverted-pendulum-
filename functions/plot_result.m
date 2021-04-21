
LW = 2;
figure()
subplot(3,2,1)
hold on
grid on
set(gcf,'color','white');
plot(save(:,1),save(:,2),'LineWidth',LW)
plot(save(:,1),save(:,1)*0 + wr(1),'k--','LineWidth',LW)
xlabel('Time [s]')
ylabel('x-position [m]')

subplot(3,2,2)
hold on
grid on
set(gcf,'color','white');
plot(save(:,1),save(:,3),'LineWidth',LW)
plot(save(:,1),save(:,1)*0 + wr(2),'k--','LineWidth',LW)
xlabel('Time [s]')
ylabel('cart speed [m/s]')

subplot(3,2,3)
hold on
grid on
set(gcf,'color','white');
plot(save(:,1),save(:,4),'LineWidth',LW)
plot(save(:,1),save(:,1)*0 + wr(3),'k--','LineWidth',LW)
xlabel('Time [s]')
ylabel('Pole angle [rad]')

subplot(3,2,4)
hold on
grid on
set(gcf,'color','white');
plot(save(:,1),save(:,5),'LineWidth',LW)
plot(save(:,1),save(:,1)*0 + wr(4),'k--','LineWidth',LW)
xlabel('Time [s]')
ylabel('Pole angular velocity [rad/s]')

subplot(3,2,[5,6])
hold on
grid on
set(gcf,'color','white');
plot(save(:,1),save(:,6),'LineWidth',LW)
xlabel('Time [s]')
ylabel('Input')

