clear all;
close all;
clc;

draw_on = 1;

m = 5; M = 25; L = 10; g = -9.81; d = 1;
box_length = 5;
radius = 1;
%% Simulate closed-loop system
dt = 0.1;
plot_dt = 0.1;
tf = 30;
save = [];
x = [-2; 0; pi+0.5; 0]; % initial condition
wr = [1; 0; pi; 0]; % reference position
u = 0;

obs = [0,0,0.00001];