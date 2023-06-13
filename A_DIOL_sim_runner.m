%% Simulation of an A-DIOL controlled bicopter
%

clc
close all
clear all

%% Printing properties

set(0, 'DefaultTextFontSize', 16); 
set(0, 'DefaultTextInterpreter', 'latex');
set(0, 'DefaultLineLineWidth', 2);
set(0, 'DefaultAxesFontSize', 16);

%% Simulation setup

% Bicopter trajectory (generate trajectories with 'trajectory_csv_generator.m')

% Ellipse trajectory
% if (~isfile('r1_traj_Ellipse.csv') || ~isfile('r2_traj_Ellipse.csv'))
%     trajectory_csv_generator;
% end
% r1_ref = readmatrix('r1_traj_Ellipse.csv');
% r2_ref = readmatrix('r2_traj_Ellipse.csv');

% Hilbert curve trajectory
if (~isfile('r1_traj_Hilbert.csv') || ~isfile('r2_traj_Hilbert.csv'))
    trajectory_csv_generator;
end
r1_ref = readmatrix('r1_traj_Hilbert.csv');
r2_ref = readmatrix('r2_traj_Hilbert.csv');

% Physical parameters
m = 1;
J = 0.05;
g = 9.81;
x0 = [0.01; 0.00; 0.03; 0.02; 0.01; 0.00];

% Estimator hyperparameters
gamma=10;
lambda = 80;
c_1 = 6;
c_2 = 3;
alpha_1 = 0.2;
alpha_2 = 1.2;

% Estimator states initial values
Psi_int_0 = [0;0;0;0;0;0];
m_hat_0 = 0.5; J_hat_0 = 0.1;
Theta_hat_0 = [1/m_hat_0; 1/J_hat_0];
xf_0 = x0;
Phif_0 = zeros(6,2);
Xbar_0 = zeros(2,1);
Phibar_0 = ones(2,2);

% Obtaining controller hyperparameters (Linear Controller Gain Computation)
A = [zeros(3,1),eye(3),zeros(3,4);zeros(1,8);zeros(3,5),eye(3);zeros(1,8)];
B = [zeros(3,2);[1 0];zeros(3,2);[0 1]];
p = [-4.5,-4.0,-5,-5.5,-4.5,-4.0,-5,-5.5]; % Desired location of the closed loop eigenvalues
k = place(A,B,p); % Gains of the ADIOL controller for the desired closed loop eigenvalues

% Initial thrust values
u1_dot_0 = 0;
u1_0 = g*m_hat_0;

%% Simulation

Ts = 1e-4;      % Trajectory sampling time in seconds.
Tend = 110;     % Simulation final time in seconds.

out = sim('A_DIOL_BicopterSim');


%% Plotting results
Time = out.tout;

rd_1 = out.r1d;
rd_2 = out.r2d;

states = out.x;
r1 = states(:,1);
r2 = states(:,2);
theta = states(:,3);

mi_hat = out.mi_hat;
Ji_hat = out.Ji_hat;
m_hat = 1./mi_hat;
J_hat = 1./Ji_hat;

u = out.u;
u1 = u(:,1);
u2 = u(:,2);

% Plotting 2D reference and position trajectory

figure(1)

set(gcf,'color','w');

plot(r1,r2)
hold on
plot(rd_1,rd_2,'--')
hold off
grid on
xlabel('$r_1\,(\rm{m})$');
ylabel('$r_2\,(\rm{m})$');
legend({'Position','Reference'})

% Plotting bicopter actual and desired states

figure(2)

set(gcf,'color','w');

subplot(3,1,1)
plot(Time,r1)
hold on
plot(Time,rd_1,'--k')
hold off
grid on
xticklabels({})
ylabel('$r_1\,(\rm{m})$');
legend({'Actual','Desired'})

subplot(3,1,2)
plot(Time,r2)
hold on
plot(Time,rd_2,'--k')
hold off
grid on
xticklabels({})
ylabel('$r_2\,(\rm{m})$');

subplot(3,1,3)
plot(Time,theta*180/pi)
grid on
xlabel('$\rm{Time}\,{\rm (s)}$');
ylabel('$\theta\,(\rm{deg})$');

% Plotting mass and inertial parameter estimates and actual values

figure(3)

set(gcf,'color','w');

subplot(2,1,1)
plot(Time,m_hat)
hold on
plot(Time, m*ones(size(Time)),'--k')
hold off
grid on
xticklabels({})
ylabel('$\hat{m} \,(\rm{kg})$');
legend({'Estimate','Actual'})

subplot(2,1,2)
plot(Time,J_hat)
hold on
plot(Time, J*ones(size(Time)),'--k')
hold off
grid on
xlabel('$\rm{Time}\,{\rm (s)}$');
ylabel('$\hat{J} \,(\rm{kg} \cdot {\rm m}^2)$');

% Plotting control inputs from vector u (Total thrust and total moment)

figure(4)

set(gcf,'color','w');

subplot(2,1,1)
plot(Time,u1)
grid on
xticklabels({})
ylabel('$u_1\,(\rm{N})$');

subplot(2,1,2)
plot(Time,u2)
grid on
xlabel('$\rm{Time}\,{\rm (s)}$');
ylabel('$u_2\,(\rm{N} \cdot \rm{m})$');
