% Quadrotor stabilization algorithms comparison
clear all; close all; clc;

global N T QQ YY DD RR grav mm Ixx Iyy Izz I_B d0 Sg Vx0 Ay0
global k_P k_D kk_P kk_D kk_I k_3 k_2 k_1 k_0 x_d y_d z_d Ke Ksf

T = 40; % Simulation time
N = 25; % Number of differential equations

grav = 9.81;
Ke = 100; % velocity estimator gain
Ksf = 1.5; % smoothing filter gain

% === CHOOSE MODEL =======================================================%
QQ = 1; % MODEL 1 - full rigid body dynamic model w/o propeller gyro effect
% QQ = 2; % MODEL 2 - simplified rigid-body dynamic model
% QQ = 3; % MODEL 3 - more simplified rigid-body dynamic model
% QQ = 4; % MODEL 4 - linear quadrotor model
%=========================================================================%

% === CHOOSE CONTROLLER ==================================================%
% YY = 1; % linear PD control with gravity compensation
% YY = 2; % PID control with gravity compensation
% YY = 3; % Trajectory tracking control law - Z axis PID controller
% YY = 4; % Sliding mode 1st order (sign)
YY = 5; % Super-twisting
%=========================================================================%

% === CHOOSE SOLVER ======================================================%
WW = 1; % Fixed-step Runge-Kutta 4th order
% WW = 2; % ODE Runge-Kutta (variable step)
%=========================================================================%

% === CHOOSE REFERENCE ===================================================%
RR = 1; % Z step reference, X & Y = 0
% RR = 2; % Spiral trajectory
%=========================================================================%

% === CHOOSE DISTURBANCE =================================================%
% --- Occurence:
DD = 0; % without disturbance
% DD = 1; % single wind gust at T/2
% DD = 2; % four wind gusts (i) at 5+i*T/4, same direction
% DD = 3; % four wind gusts (i) at 5+i*T/4, alternating direction

% --- Shape:
% d0=1; Sg=5; % short duration, small amplitude
d0=4; Sg=0.1; % long duration, large amplitude
%=========================================================================%

% === CHOOSE INITIAL CONDITIONS ==========================================%
xx0 = zeros(1, N);
if (QQ == 1)
   xx0(4)=0*0.1; xx0(5)=0*0.1; xx0(6)=0*0.1; % initial angles
end
if (QQ == 2)||(QQ == 3)||(QQ == 4)
   xx0(7)=0*0.1; xx0(9)=0*0.1; xx0(11)=0*0.1; % initial angles
end
%=========================================================================%

% === CHOOSE QUADROTOR PARAMETERS ========================================%
% "Image Based Visual Servoing for an Autonomous Quadrotor with Adaptive Backstepping Control":
% mm = 1; Ixx = 0.62; Iyy = 0.62; Izz = 1.24; 

% "Flatness-based control of a quadrotor helicopter via feedforward linearization":
% mm=0.5; Ixx = 0.005; Iyy = 0.005; Izz = 0.009; 

% "Dynamic modeling and nonlinear control strategy for an underactuated quad rotor rotorcraft":
mm=0.6; Ixx = 0.0154; Iyy = 0.0154; Izz = 0.0309; 

% "Backstepping Control for a Quadrotor Helicopter":
% mm=2; Ixx = 1.2416; Iyy = 1.2416; Izz = 2.4832; l = 0.1; d = 0.0000001; b=0.0000008;
    % l - dist to COM; b - thrust factor; d - drag factor
    % (l, b, d) notation used in bible; 
    % values l,d,b modified from original paper data (d, /, c)

Ixy = 0; Iyz = 0; Ixz = 0;
I_B = [Ixx -Ixy -Ixz; -Ixy Iyy -Iyz; -Ixz -Iyz Izz];
%=========================================================================%


% --- Reference trajectory parameters ------------------------------------%
if (RR == 1)
    x_d = 0; y_d = 0; z_d = 1;
end
if (RR == 2)
    Vx0=0.5; Ay0=1;
end
%-------------------------------------------------------------------------%

% --- Controller parameters ----------------------------------------------%
% Poles:
pol_1 = -2;
pol_2 = -3;
pol_3 = -4;

% PD:
k_P = pol_1 * pol_2;
k_D = -(pol_1 + pol_2);

% PID:
kk_P = mm*(pol_1*pol_2 + pol_1*pol_3 + pol_2*pol_3);
kk_D = -mm*(pol_1+pol_2+pol_3); 
kk_I = -mm*pol_1*pol_2*pol_3;

% Poles:
pol_1 = -3;
pol_2 = -3;
pol_3 = -4;
pol_4 = -5;

% Control for X and Y tracking:
k_3 = -pol_1 - pol_2 - pol_3 - pol_4;
k_2 = pol_1*pol_2 + pol_2*pol_3 + pol_3*pol_4 + pol_1*pol_4 + pol_2*pol_4 + pol_3*pol_1;
k_1 = -pol_1*pol_2*pol_3 - pol_1*pol_2*pol_4 - pol_2*pol_3*pol_4 - pol_1*pol_3*pol_4; 
k_0 = pol_1*pol_2*pol_3*pol_4;
%-------------------------------------------------------------------------%

output(T, QQ, YY, WW, RR, DD);

if (WW == 1)
% --- Fixed-step Runge-Kutta 4th order -----------------------------------%
tspan = [0 T]; Nstep = 10000; DeltaT = T/Nstep;
[t, y] = rk4(@QuadroHB, tspan, xx0, DeltaT);
end
%-------------------------------------------------------------------------%

if (WW == 2)
% --- ODE Runge-Kutta (variable step) ----------------------------------%
options = odeset('RelTol',1e-6,'AbsTol',1e-6);
[t,y] = ode45('QuadroHB',[0 T],xx0,options);
%-------------------------------------------------------------------------%
end

fprintf('Done! \n');

% === PLOTS ==============================================================%
set(0, 'DefaultFigurePosition', [1367 -281 1920 973]); % set all plots position to center of secondary monitor at home

% --- Reference trajectories for ploting ---------------------------------%
if (RR == 1) 
    x_d = zeros(size(t));
    y_d = zeros(size(t));
    z_d = zeros(size(t));
    z_d = [ones(size(t(1:round(3*end/4)))); zeros(size(t(1:round(1*end/4))))];    
end
if (RR == 2)
    x_d = -Ay0*0 + Ay0*cos(Vx0*t);
    y_d = Ay0*sin(Vx0*t);
    z_d = Vx0*t;
end     

if DD == 0
d_0 = 0*t;
end
if DD == 1
d_0 = d0*exp(-Sg*(t-T/2).^2);
end
if DD == 2
d_0 = d0*exp(-Sg*(t+5-1*T/4).^2) + d0*exp(-Sg*(t+5-2*T/4).^2) + d0*exp(-Sg*(t+5-3*T/4).^2) + d0*exp(-Sg*(t+5-4*T/4).^2);
end
if DD == 3
d_0 = d0*exp(-Sg*(t+5-1*T/4).^2) - d0*exp(-Sg*(t+5-2*T/4).^2) + d0*exp(-Sg*(t+5-3*T/4).^2) - d0*exp(-Sg*(t+5-4*T/4).^2);
end
%-------------------------------------------------------------------------%

if (QQ == 1)
% --- MODEL 1 ------------------------------------------------------------%
figure(1)
subplot(2,3,1), plot(t,y(:,1),'b', t, x_d,'r:', 'linewidth',4), ylabel('x (m)','FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times'), 
subplot(2,3,2), plot(t,y(:,2),'b', t, y_d,'r:', 'linewidth',4), ylabel('y (m)','FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times'), 
subplot(2,3,3), plot(t,y(:,3),'b', t, z_d,'r:', 'linewidth',4), ylabel('z (m)','FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times'), %axis([0 10 0 1.2])
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
de_z_est = diff(y(:,17))./diff(t);
dZ = diff(y(:,3))./diff(t);
td=t(1:(length(t)-1));

figure(3)
subplot(2,2,1), plot(td,F,'b', 'linewidth',3), ylabel('F_z (N)','FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times'), %axis([0 5 9 15])
subplot(2,2,2), plot(td,T1,'b', 'linewidth',3), ylabel('\tau_1 (Nm)','FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times'), %axis([0 5 -10 5])
subplot(2,2,3), plot(td,T2,'b', 'linewidth',3), ylabel('\tau_2 (Nm)','FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times'), %axis([0 5 -1 5])
subplot(2,2,4), plot(td,T3,'b', 'linewidth',3), ylabel('\tau_3 (Nm)','FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times'), %axis([0 5 -1 1])


% Estimated velocity
figure(4) % usporedi izlaz filtra za estimaciju brzine(od greske) i prave vrijednosti
subplot(2,1,1), plot(td, de_z_est,'b-', td, dZ, 'r:', 'linewidth',4), ylabel('dZ estimated','FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times')
subplot(2,1,2), plot(t, y(:, 9), 'linewidth',4), ylabel('dZ from model','FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times')
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
de_z_est = diff(y(:,17))./diff(t);
dZ = diff(y(:,5))./diff(t);
td=t(1:(length(t)-1));

figure(3)
subplot(2,2,1), plot(td,F,'b', 'linewidth',3), ylabel('F_z (N)','FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times'), %axis([0 5 9 15])
subplot(2,2,2), plot(td,T1,'b', 'linewidth',3), ylabel('\tau_1 (Nm)','FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times'), %axis([0 5 -10 5])
subplot(2,2,3), plot(td,T2,'b', 'linewidth',3), ylabel('\tau_2 (Nm)','FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times'), %axis([0 5 -1 5])
subplot(2,2,4), plot(td,T3,'b', 'linewidth',3), ylabel('\tau_3 (Nm)','FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times'), %axis([0 5 -1 1])

% Estimated velocity
figure(4) % usporedi izlaz filtra za estimaciju brzine(od greske) i prave vrijednosti
subplot(2,1,1), plot(td, de_z_est,'b-', td, dZ, 'r:', 'linewidth',4), ylabel('Z axis velocity (m/s)','FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times'),
legend('Estimated velocity', 'Exact velocity');
subplot(2,1,2), plot(t, y(:, 6), 'linewidth',4), ylabel('Z axis velocity (m/s)','FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times'),
legend('Pure derivative velocity');
%--------------------------------------------------------------%
end

% Filtered reference
figure(5)
subplot(2,1,1), plot(t, y(:, 18), 'b-', t, z_d, 'r:', 'Linewidth', 4), ylabel('z_d (m)','FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times'),
legend('1st order smoothing filter', 'Z position reference');
subplot(2,1,2), plot(t, y(:, 19), 'b-', t, z_d, 'r:', 'Linewidth', 4), ylabel('z_d (m)','FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times'),
legend('2nd order smoothing filter', 'Z position reference');

% Disturbance
figure(6)
plot(t, d_0, 'b-', 'Linewidth', 4), ylabel('Wind gust', 'FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times');

if (YY == 4) || (YY == 5)
    s = diff(y(:,24))./diff(t);
    figure(20)
    plot(td, s, 'b-', 'Linewidth', 4), ylabel('error', 'FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times'), 
    legend('Sliding variable s'), grid 'on', axis([0 20 -0.8 0.8]);
end
%=========================================================================%

