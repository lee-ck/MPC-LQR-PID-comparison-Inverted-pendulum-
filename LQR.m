addpath('functions')
Init_simul

b = 1; % Pendulum up

A = [0             1                   0     0;
     0          -d/M             b*m*g/M     0;
     0             0                   0     1;
     0     -b*d/(M*L)    -b*(m+M)*g/(M*L)    0];
 
B = [0;
     1/M; 
     0; 
     b*1/(M*L)];

%% Design LQR controller
Q = eye(4); % 4x4 identify matrix
R = .0001;
K = lqr(A,B,Q,R);

for time = 0:dt:tf
    save = [save; time, x', u];
    %% LQR
    u= - K*(x - wr); % control law
    

    %% Update        
    dx = pendcart(x,m,M,L,g,d,u);    
    x = x + dx*dt;

    if mod(time,plot_dt) < 0.001 && draw_on
        plot_drawnow
    end
end
plot_result
sgtitle('LQR') 
