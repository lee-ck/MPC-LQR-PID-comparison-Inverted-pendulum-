addpath('functions')
Init_simul

Px = -5;
Dx = -20;
Pt = 1000;
Dt = 1000;

for time = 0:dt:tf
    save = [save; time, x', u];
    %% PID
    ux = (wr(1) - x(1)) * Px + (wr(2) - x(2)) * Dx;
    ut = (wr(3) - x(3)) * Pt + (wr(4) - x(4)) * Dt;
    u = ux + ut;
    

    %% Update        
    dx = pendcart(x,m,M,L,g,d,u);    
    x = x + dx*dt;

    if mod(time,plot_dt) < 0.001 && draw_on
        plot_drawnow
    end
end
plot_result()
sgtitle('PID') 

