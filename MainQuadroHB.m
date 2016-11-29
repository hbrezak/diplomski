% Quadrotor stabilization algorithms comparison
clear all; close all; clc;

global N QQ grav mm Ixx Iyy Izz I_B

T = 40; % Simulation time
N = 16; % Number of differential equations

grav = 9.81;

% === CHOOSE MODEL =======================================================%
QQ = 1; % MODEL 1 - full rigid body dynamic model w/o propeller gyro effect

%=========================================================================%


% === CHOOSE SOLVER ======================================================%
WW = 1; % RK4

%=========================================================================%


% === CHOOSE QUADROTOR PARAMETERS ========================================%
% "Image Based Visual Servoing for an Autonomous Quadrotor with Adaptive Backstepping Control":
mm = 1; Ixx = 0.62; Iyy = 0.62; Izz = 1.24; 

% "Flatness-based control of a quadrotor helicopter via feedforward linearization":
% mm=0.5; Ixx = 0.005; Iyy = 0.005; Izz = 0.009; 

% "Dynamic modeling and nonlinear control strategy for an underactuated quad rotor rotorcraft":
% mm=0.6; Ixx = 0.0154; Iyy = 0.0154; Izz = 0.0309; 

% "Backstepping Control for a Quadrotor Helicopter":
% mm=2; Ixx = 1.2416; Iyy = 1.2416; Izz = 2.4832; % d = 0.2m, c = 0.01m %

Ixy = 0; Iyz = 0; Ixz = 0;
I_B = [Ixx -Ixy -Ixz; -Ixy Iyy -Iyz; -Ixz -Iyz Izz];
%=========================================================================%


% === CHOOSE DISTURBANCE =================================================%

%=========================================================================%


% --- Reference trajectory parameters ------------------------------------%

%-------------------------------------------------------------------------%


% --- Controller parameters ----------------------------------------------%


%-------------------------------------------------------------------------%
xx0 = zeros(1, N);

if (WW == 1)
% Fixed-step Runge-Kutta 4th order
tspan = [0 T]; Nstep = 10000; DeltaT = T/Nstep:
[t, y] = rk4(@QuadroHB, tspan, xx0, DeltaT);
end


% --- Reference trajectories ---------------------------------------------%

%-------------------------------------------------------------------------%

% --- PLOTS --------------------------------------------------------------%


%-------------------------------------------------------------------------%