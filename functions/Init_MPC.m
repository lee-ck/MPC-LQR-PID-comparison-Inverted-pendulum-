addpath('C:\Users\user\Desktop\CasADi\casadi-windows-matlabR2016a-v3.5.5\')
import casadi.*
Ts = 5;
Num = Ts/dt;

%% States
pos = SX.sym('pos'); 
vel = SX.sym('vel'); 
theta = SX.sym('theta');
ang_vel= SX.sym('ang_vel');
states = [pos;vel;theta;ang_vel]; 
n_states = length(states);

force = SX.sym('force');
controls = force;
n_controls = length(controls);
U = SX.sym('U',n_controls,Num); 
P = SX.sym('P',n_states*2 + 3);
X = SX.sym('X',n_states,(Num+1));
objective = 0; % Objective function
constraints = [];  % constraints vector

W1 = diag([10,1,30,1]);
W2 = 0.0001;
W3 = diag([10,1,30,1])*10;
st  = X(:,1); % initial state
constraints = [constraints;st-P(1:n_states)]; % initial condition constraints
%% calculate objective function
for k = 1:Num
st = X(:,k);  
con = U(:,k); 
%% Cost function
objective = objective + (st-P(n_states+1:n_states*2))'*W1*(st-P(n_states+1:n_states*2)) + con'*W2*con;

%% Ship dynamics
st_next = X(:,k+1);
xdot = pendcart_MPC(st,m,M,L,g,d,con);
st_next_euler = st + (dt*xdot);
constraints = [constraints;st_next-st_next_euler]; % compute constraints
end
st = X(:,Num+1);  
objective = objective + (st-P(n_states+1:n_states*2))'*W3*(st-P(n_states+1:n_states*2));
constraints = [constraints;st_next-st_next_euler]; % compute constraints

for k = 1:Num+1
st = X(:,k);
obs_x = P(9);
obs_y = P(10);
obs_r = P(11);
constraints = [constraints;(st(1)+L*cos(st(3)-pi/2) - obs_x)^2 + (box_length/2+L*sin(st(3)-pi/2) - obs_y)^2 - (obs_r + radius)^2]; % compute constraints
end

% make the decision variable one column  vector
OPT_variables = [reshape(X,n_states*(Num+1),1);reshape(U,n_controls*Num,1)];

nlp_prob = struct('f', objective, 'x', OPT_variables, 'g', constraints, 'p', P);

opts = struct;
opts.ipopt.max_iter = 10;
opts.ipopt.print_level = 1; % 0 ~ 3
opts.print_time = 0;
opts.ipopt.acceptable_tol = 1e-8;
opts.ipopt.acceptable_obj_change_tol = 1e-8;

solver = nlpsol('solver', 'ipopt', nlp_prob,opts);
%% Equaility contraints : Dyanmic equations 
args.lbg(1:(n_states)*(Num+2)) = -1e-10;  % -1e-20   % Equality constraints
args.ubg(1:(n_states)*(Num+2)) =  1e-10;  %  1e-20   % Equality constraints
args.lbg((n_states)*(Num+2)+1:(n_states+1)*(Num+2)-1) = -1e-10;  % -1e-20   % Equality constraints
args.ubg((n_states)*(Num+2)+1:(n_states+1)*(Num+2)-1) = 1e10;  % -1e-20   % Equality constraints

%% Inequaility contraints : States & Inputs
args.lbx(1:n_states:n_states*(Num+1),1) =  -1e10;
args.ubx(1:n_states:n_states*(Num+1),1) =   1e10;
args.lbx(2:n_states:n_states*(Num+1),1) =  -1e10;
args.ubx(2:n_states:n_states*(Num+1),1) =   1e10;
args.lbx(3:n_states:n_states*(Num+1),1) =   pi/2;
args.ubx(3:n_states:n_states*(Num+1),1) =   3*pi/2;
args.lbx(4:n_states:n_states*(Num+1),1) =  -1e10;
args.ubx(4:n_states:n_states*(Num+1),1) =   1e10;
args.lbx(4*(Num+1)+1:1:n_states*(Num+1)+1*Num,1) = -3e3;
args.ubx(4*(Num+1)+1:1:n_states*(Num+1)+1*Num,1) =  3e3;


disp('*************** MPC setting ***************')
disp(strcat('Prediction Num :` ' , num2str(Num)))
disp(strcat('Prediction Ts :` ' , num2str(Ts) ,'sec'))



function xdot = pendcart_MPC(x,m,M,L,g,d,u)
Sx = sin(x(3));
Cx = cos(x(3));
D = m*L*L*(M+m*(1-Cx^2));
xdot = [x(2);...
(1/D)*(-m^2*L^2*g*Cx*Sx+m*L^2*(m*L*x(4)^2*Sx-d*x(2)))+m*L*L*(1/D)*u;...
x(4);...
(1/D)*((m+M)*m*g*L*Sx - m*L*Cx*(m*L*x(4)^2*Sx - d*x(2))) - m*L*Cx*(1/D)*u];
end

