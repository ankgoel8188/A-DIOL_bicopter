%% Simulation of an A-DIOL discrete-time controlled bicopter
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

r1_ref_0 = (r1_ref(1,2:6))';
r2_ref_0 = (r2_ref(1,2:6))';

% Physical parameters
Ts = 0.01;
m = 1;
J = 0.05;
g = 9.81;
x0 = [0.01; 0.00; 0.03; 0.02; 0.01; 0.00];

% Estimator hyperparameters
gamma=10;
lambda = 40;
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


% Obtaining discrete-time controller hyperparameters
%p = [-4.5,-4.0,-5,-5.5,-4.5,-4.0,-5,-5.5]; % Desired location of the
%continuous-time, closed loop eigenvalues (previous values)
p = [-1.5,-1.0,-0.5,-2.5,-1.5,-1.0,-0.5,-2.5]; % Desired location of the continuous-time, closed loop eigenvalues

Ad1 = eye(4) + Ts*[zeros(3,1),eye(3);zeros(1,4)] + (Ts^2)/2*[zeros(2,2),eye(2);zeros(2,4)] + (Ts^3)/6*[zeros(1,3),1;zeros(3,4)];
Ad2 = Ad1;
Bd1 = [(Ts^4)/24; (Ts^3)/6; (Ts^2)/2; Ts];
Bd2 = Bd1;

Ad = [Ad1 zeros(4); zeros(4) Ad2];
Bd = [Bd1 zeros(4,1); zeros(4,1) Bd2];
pd = exp(p.*Ts);
kd = place(Ad,Bd,pd); % Gains of the ADIOL discrete-time controller for the desired closed loop eigenvalues

% Initial thrust values
u1_dot_0 = 0;
u1_0 = g*m_hat_0;
u_0 = [u1_0; 0];

%% Simulation
Ts_sim = 1e-4;      % Trajectory sampling time in seconds.
Tend = 110;     % Simulation final time in seconds.

out = sim('DT_A_DIOL_BicopterSim');


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
%plot(Time,rd_1,'--k')
plot(r1_ref(:,1),r1_ref(:,2),'--k')
hold off
grid on
xticklabels({})
ylabel('$r_1\,(\rm{m})$');
legend({'Actual','Desired'})

subplot(3,1,2)
plot(Time,r2)
hold on
%plot(Time,rd_2,'--k')
plot(r2_ref(:,1),r2_ref(:,2),'--k')
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
stairs(Time,m_hat)
hold on
plot(Time, m*ones(size(Time)),'--k')
hold off
grid on
xticklabels({})
ylabel('$\hat{m} \,(\rm{kg})$');
legend({'Estimate','Actual'})

subplot(2,1,2)
stairs(Time,J_hat)
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
stairs(Time,u1)
grid on
xticklabels({})
ylabel('$u_1\,(\rm{N})$');

subplot(2,1,2)
stairs(Time,u2)
grid on
xlabel('$\rm{Time}\,{\rm (s)}$');
ylabel('$u_2\,(\rm{N} \cdot \rm{m})$');
