addpath('functions')
Init_simul
Init_MPC

input.u0 = zeros(Num,n_controls);
input.X0 = repmat(x,1,Num+1)';             
for time = 0:dt:tf
    save = [save; time, x', u];

    %% MPC
    args.p(1:n_states) = x';
    args.p(n_states+1:2*n_states) = wr';
    args.p(9:11) = obs;
    args.x0  = [reshape(input.X0',n_states*(Num+1),1);reshape(input.u0',n_controls*Num,1)];
    sol = solver('x0', args.x0,'lbx',args.lbx,'ubx', args.ubx,'lbg', args.lbg, 'ubg', args.ubg,'p',args.p);
    usol = reshape(full(sol.x(n_states*(Num+1)+1:end))',n_controls,Num)'; % get controls only from the solution
    xsol= reshape(full(sol.x(1:n_states*(Num+1)))',n_states,Num+1)'; % get solution TRAJECTORY

    input.X0 = [xsol(2:end,:);xsol(end,:)];
    input.u0 = [usol(2:end,:);usol(end,:)];

    output.x = xsol;
    output.u = usol;

    u = usol(1);
    %% Update        
    dx = pendcart(x,m,M,L,g,d,u);    
    x = x + dx*dt;
    
    if mod(time,plot_dt) < 0.001 && draw_on       
        plot_drawnow
%         plot(xsol(:,1)+L.*cos(xsol(:,3) - pi/2),box_length/2+L.*sin(xsol(:,3) - pi/2),'k--','LineWidth',1)
%         plot(xsol(:,1),xsol(:,1)*0 + box_length/2,'b--','LineWidth',1)
    end
end
% plot(save(:,2)+L.*cos(save(:,4) - pi/2),box_length/2+L.*sin(save(:,4) - pi/2),'k--','LineWidth',1)

plot_result
sgtitle('NMPC') 
