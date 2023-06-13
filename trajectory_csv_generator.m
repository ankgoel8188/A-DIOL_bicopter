close all
clear all
clc

Ts = 1e-4;      % Trajectory sampling time in seconds.
Tend = 110;     % Simulation final time in seconds.

%% Elliptical curve trajectory csv generator

% Trajectory parameters
a1 = 5;
a2 = 3;

out = sim('Ellipse_Trajectory_Generator');

r1 = out.r1_traj;
r2 = out.r2_traj;
t = out.tt;

writematrix([t r1],'r1_traj_Ellipse.csv');
writematrix([t r2],'r2_traj_Ellipse.csv');

%% Hilbert curve trajectory csv generator

% Trajectory parameters
alpha = 2;

out = sim('Hilbert_Curve_Trajectory_Generator');

r1 = out.r1_traj;
r2 = out.r2_traj;
t = out.tt;

writematrix([t r1],'r1_traj_Hilbert.csv');
writematrix([t r2],'r2_traj_Hilbert.csv');