% Quadrotor stabilization algorithms comparison
clear all; close all; clc;

global N T QQ YY grav mm Ixx Iyy Izz I_B d0 Sg
global k_P k_D x_d y_d z_d Kest Kf1

T = 40; % Simulation time
N = 18; % Number of differential equations

grav = 9.81;
Kest = 100;
Kf1 = 1.2;

% === CHOOSE MODEL =======================================================%
% QQ = 1; % MODEL 1 - full rigid body dynamic model w/o propeller gyro effect
% QQ = 2; % MODEL 2 (simplified rigid-body dynamic model)
% QQ = 3; % MODEL 3 (more simplified rigid-body dynamic model)
QQ = 4; % MODEL 4 (linear quadrotor model)
%=========================================================================%

% === CHOOSE CONTROLLER ==================================================%
YY = 1; % linear PD control

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
% mm=2; Ixx = 1.2416; Iyy = 1.2416; Izz = 2.4832; l = 0.1; d = 0.0000001; b=0.0000008;
    % l - dist to COM; b - thrust factor; d - drag factor
    % (l, b, d) notation used in bible; (d, /, c)
    % values l,d,b modified from original paper data

Ixy = 0; Iyz = 0; Ixz = 0;
I_B = [Ixx -Ixy -Ixz; -Ixy Iyy -Iyz; -Ixz -Iyz Izz];
%=========================================================================%


% === CHOOSE DISTURBANCE =================================================%

%=========================================================================%
d0=4; Sg=0.1; % disturbance parameters

% --- Reference trajectory parameters ------------------------------------%
x_d = 0; y_d = 0; z_d = 1;
%-------------------------------------------------------------------------%


% --- Controller parameters ----------------------------------------------%
% Polovi:
pol_1 = -2;
pol_2 = -3;
pol_3 = -4;

k_P = pol_1 * pol_2;
k_D = -(pol_1 + pol_2);

%-------------------------------------------------------------------------%

% --- Initial conditions -------------------------------------------------%
xx0 = zeros(1, N);
if (QQ == 1)
   xx0(4)=0*0.1; xx0(5)=0*0.1; xx0(6)=0*0.1; % initial angles
end
if (QQ == 2)||(QQ == 3)||(QQ == 4)
   xx0(7)=1*0.1; xx0(9)=1*0.1; xx0(11)=1*0.1; % initial angles
end
%-------------------------------------------------------------------------%


if (WW == 1)
% Fixed-step Runge-Kutta 4th order
tspan = [0 T]; Nstep = 10000; DeltaT = T/Nstep;
[t, y] = rk4(@QuadroHB, tspan, xx0, DeltaT);
end


% === PLOTS ==============================================================%
set(0, 'DefaultFigurePosition', [1367 -281 1920 973]); % set all plots position to center of secondary monitor at home

% --- Reference trajectories ---------------------------------------------%

z_d = zeros(size(t)); 
z_d = [ones(size(t(1:round(3*end/4)))); zeros(size(t(1:round(1*end/4))))]; 

x_d = zeros(size(t)); 
y_d = zeros(size(t)); 

%-------------------------------------------------------------------------%

if (QQ == 1)
% --- MODEL 1 ------------------------------------------------------------%
figure(1)
subplot(2,3,1), plot(t,y(:,1),'b', t,x_d,'r:', 'linewidth',4), ylabel('x (m)','FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times'), 
subplot(2,3,2), plot(t,y(:,2),'b', t,y_d,'r:', 'linewidth',4), ylabel('y (m)','FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times'), 
subplot(2,3,3), plot(t,y(:,3),'b', t,z_d,'r:', 'linewidth',4), ylabel('z (m)','FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times'), %axis([0 10 0 1.2])
subplot(2,3,4), plot(t,y(:,4),'b', 'linewidth',4), ylabel('\phi (rad)','FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times'), 
subplot(2,3,5), plot(t,y(:,5),'b', 'linewidth',4), ylabel('\theta (rad)','FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times'), 
subplot(2,3,6), plot(t,y(:,6),'b', 'linewidth',4), ylabel('\psi (rad)','FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times'), 

% Errors
figure(2)
subplot(2,3,1), semilogy(t,abs(y(:,1)-x_d), 'b', 'linewidth',4), ylabel('|x-x_d| (m)','FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times'), 
subplot(2,3,2), semilogy(t,abs(y(:,2)-y_d), 'b', 'linewidth',4), ylabel('|y-y_d| (m)','FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times'), 
subplot(2,3,3), semilogy(t,abs(y(:,3)-z_d), 'b', 'linewidth',4), ylabel('|z-z_d| (m)','FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times'), 

% Thrust force and torques
F=diff(y(:,13))./diff(t);
T1=diff(y(:,14))./diff(t);
T2=diff(y(:,15))./diff(t);
T3=diff(y(:,16))./diff(t);
d1e_z_est = diff(y(:,17))./diff(t);
d1Z = diff(y(:,3))./diff(t);
td=t(1:(length(t)-1));

figure(3)
subplot(2,2,1), plot(td,F,'b', 'linewidth',3), ylabel('F_z (N)','FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times'), %axis([0 5 9 15])
subplot(2,2,2), plot(td,T1,'b', 'linewidth',3), ylabel('\tau_1 (Nm)','FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times'), %axis([0 5 -10 5])
subplot(2,2,3), plot(td,T2,'b', 'linewidth',3), ylabel('\tau_2 (Nm)','FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times'), %axis([0 5 -1 5])
subplot(2,2,4), plot(td,T3,'b', 'linewidth',3), ylabel('\tau_3 (Nm)','FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times'), %axis([0 5 -1 1])


% Estimated velocity
figure(4) % usporedi izlaz filtra za estimaciju brzine(od greske) i prave vrijednosti
subplot(2,1,1), plot(td, d1e_z_est,'b-', td, d1Z, 'r:', 'linewidth',4), ylabel('d1Z estimated','FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times')
subplot(2,1,2), plot(t, y(:, 9), 'linewidth',4), ylabel('d1Z from model','FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times')
%-------------------------------------------------------------------------%
end

if (QQ == 2)||(QQ == 3)||(QQ == 4)
% --- MODEL 2 - MODEL 3 - MODEL 4 ----------------------------------------%
figure(1)
subplot(2,3,1), plot(t,y(:,1),'b', t, x_d,'r:', 'linewidth',4), ylabel('x (m)','FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times'), 
subplot(2,3,2), plot(t,y(:,3),'b', t, y_d,'r:', 'linewidth',4), ylabel('y (m)','FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times'), 
subplot(2,3,3), plot(t,y(:,5),'b', t, z_d,'r:', 'linewidth',4), ylabel('z (m)','FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times'), 
subplot(2,3,4), plot(t,y(:,7),'b', 'linewidth',4), ylabel('\phi (rad)','FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times'), 
subplot(2,3,5), plot(t,y(:,9),'b', 'linewidth',4), ylabel('\theta (rad)','FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times'), 
subplot(2,3,6), plot(t,y(:,11),'b', 'linewidth',4), ylabel('\psi (rad)','FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times'), 

% Errors
figure(2)
subplot(2,3,1), semilogy(t,abs(y(:,1)-x_d), 'b', 'linewidth',4), ylabel('|x-x_d| (m)','FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times'), 
subplot(2,3,2), semilogy(t,abs(y(:,3)-y_d), 'b', 'linewidth',4), ylabel('|y-y_d| (m)','FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times'), 
subplot(2,3,3), semilogy(t,abs(y(:,5)-z_d), 'b', 'linewidth',4), ylabel('|z-z_d| (m)','FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times'), 

% Thrust force and torques
F=diff(y(:,13))./diff(t);
T1=diff(y(:,14))./diff(t);
T2=diff(y(:,15))./diff(t);
T3=diff(y(:,16))./diff(t);
d1e_z_est = diff(y(:,17))./diff(t);
d1Z = diff(y(:,5))./diff(t);
td=t(1:(length(t)-1));

figure(3)
subplot(2,2,1), plot(td,F,'b', 'linewidth',3), ylabel('F_z (N)','FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times'), %axis([0 5 9 15])
subplot(2,2,2), plot(td,T1,'b', 'linewidth',3), ylabel('\tau_1 (Nm)','FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times'), %axis([0 5 -10 5])
subplot(2,2,3), plot(td,T2,'b', 'linewidth',3), ylabel('\tau_2 (Nm)','FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times'), %axis([0 5 -1 5])
subplot(2,2,4), plot(td,T3,'b', 'linewidth',3), ylabel('\tau_3 (Nm)','FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times'), %axis([0 5 -1 1])

% Estimated velocity
figure(4) % usporedi izlaz filtra za estimaciju brzine(od greske) i prave vrijednosti
subplot(2,1,1), plot(td, d1e_z_est,'b-', td, d1Z, 'r:', 'linewidth',4), ylabel('d1Z estimated','FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times')
subplot(2,1,2), plot(t, y(:, 6), 'linewidth',4), ylabel('d1Z from model','FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times')

%--------------------------------------------------------------%
end


%==========================================================================%