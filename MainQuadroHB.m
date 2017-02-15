% === QUADROTOR STABILIZATION ALGORITHMS COMPARISON ======================%
clear all; clc;

global N T QQ YY DD RR SF EE SAT
global grav mm Ixx Iyy Izz I_B d0 Sg Vx0 Ay0 a1 a2 w1 w2 stepAmp
global k_P k_D kk_P kk_D kk_I k_3 k_2 k_1 k_0 Ke_lin Ke_st Ksf rho u kg
global E_B inv_E_B AngVel_limit Phi_d Theta_d Psi_d

T = 20; % Simulation time
N = 59; % Number of differential equations

% Define constant parameters
grav = 9.81;
Ke_lin = 30; % linear velocity estimator gain
Ke_st = 8; % super-twisting velocity estimator gain
Ksf = 1.5; % smoothing filter %could be higher == faster response
rho = 80; % larger - faster response
u = 5; % larger - sharper change
kg = 28; % max. thrust for EMAX RS2205@12V w/ HQ5045BN [Newtons]

% === CHOOSE MODEL =======================================================%
% QQ = 1; % MODEL 1 - full rigid body dynamic model w/o propeller gyro effect
% QQ = 2; % MODEL 2 - simplified rigid-body dynamic model
% QQ = 3; % MODEL 3 - more simplified rigid-body dynamic model
QQ = 4; % MODEL 4 - linear quadrotor model
%=========================================================================%


% === CHOOSE CONTROLLER ==================================================%
% YY = 1; % linear PD control with gravity compensation
% YY = 2; % PID control with gravity compensation
% YY = 3; % Sliding mode 1st order (Z axis)
% YY = 4; % Super-twisting (2nd order sliding mode) algorithm (Z axis)

% YY = 5; % PID trajectory tracking control law
% YY = 6; % 1-SM trajectory tracking control law
% YY = 7; % Super-twisting trajectory tracking control law
YY = 8;
%=========================================================================%


% === CHOOSE SOLVER ======================================================%
WW = 1; % Fixed-step Runge-Kutta 4th order
% WW = 2; % ODE Runge-Kutta (variable step)
%=========================================================================%


% === CHOOSE REFERENCE ===================================================%
RR = 0;
% RR = 1; % Z step reference, X & Y = 0
% RR = 2; % Spiral trajectory
% RR = 3; % based on sinusoidal function, repeats after 4 sec
%=========================================================================%


% === CHOOSE REFERENCE SMOOTHING FILTER ==================================%
SF = 0; % Z reference w/o smoothing filter
% SF = 1; % Z reference w/ smoothing filter 1st order
% SF = 2; % Z reference w/ smoothing filter 2nd order
% SF = 3; % Z reference w/ nonlinear saturated smoothing filter
%=========================================================================%


% === CHOOSE ERROR DERIVATIVE ESTIMATOR ==================================%
EE = 0; % w/o estimator
% EE = 1; % linear estimator
% EE = 2; % super-twisting estimator
%=========================================================================%


% === CHOOSE DISTURBANCE =================================================%
% --- Type:
DD = 0; % without disturbance
% DD = 1; % single wind gust at T/2
% DD = 2; % four wind gusts (i) at 5+i*T/4, same direction
% DD = 3; % four wind gusts (i) at 5+i*T/4, alternating direction
% DD = 4; % rapid alternating wave disturbance

% --- Shape:
d0=1; Sg=5; % short duration, small amplitude
% d0=4; Sg=0.1; % long duration, large amplitude; simulates wind gusts of approx. 46 km/h
% d0=2; Sg=0.3; % estimate of wind gust 60 km/h, duration 5 sec (if m = 0.6)
%=========================================================================%


% === CHOOSE INITIAL CONDITIONS ==========================================%
xx0 = zeros(1, N); % Set all initial conditions to zero

% Define exceptions:
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
% mm = 0.5; Ixx = 0.005; Iyy = 0.005; Izz = 0.009;

% "Dynamic modeling and nonlinear control strategy for an underactuated quad rotor rotorcraft":
mm = 0.6; Ixx = 0.0154; Iyy = 0.0154; Izz = 0.0309;

% "Backstepping Control for a Quadrotor Helicopter":
% mm = 2; Ixx = 1.2416; Iyy = 1.2416; Izz = 2.4832; l = 0.1; d = 0.0000001; b=0.0000008;
% l - dist to COM; b - thrust factor; d - drag factor
% (l, b, d) notation used in thesis;
% values l,d,b modified from original paper data (d, /, c)

% "Robust output tracking control of a quadrotor in the presence of
% external disturbances" (prof. Kasac, FAMENA 2013.):
% mm = 1; Ixx = 0.62; Iyy = 0.62; Izz = 1.24;

% Solidworks data for 250 class quad
% mm = 0.65; Ixx = 0.002821; Iyy = 0.004446; Izz = 0.001825;

Ixy = 0; Iyz = 0; Ixz = 0;
I_B = [Ixx -Ixy -Ixz; -Ixy Iyy -Iyz; -Ixz -Iyz Izz];
l = 0.125; % 250 class quadrotor frame
%=========================================================================%


% === CHOOSE MOTOR SATURATION ============================================%
% SAT = true;
SAT = false;
%=========================================================================%


% === MOTOR PARAMETERS ===================================================%
% Calculated from experimental values for EMAX RS2205@12V w/ HQ5045 BN prop
% Max. performance: I = 20.7A, F_T = 0.712*9.81, P = 248.4W, RPM = 20080;
b = 1.58*10^-6; % thrust factor
d = 2.67*10^-8; % drag factor
AngVel_limit = 2100; % [rad/s]; calculated maximum is 2102.8 rad/s

E_B = [b b b b; l*b -l*b -l*b l*b; -l*b l*b -l*b l*b; d d -d -d];
inv_E_B = inv(E_B);
%=========================================================================%

% Output selected parameters to console
output(T, QQ, YY, WW, RR, DD, SF, EE, SAT, mm, Ixx, Iyy, Izz, b, d, l, AngVel_limit); 

% --- Reference trajectory parameters ------------------------------------%
if (RR == 1)
    stepAmp = 1;
    x_d = 0; y_d = 0; z_d = 1;
end
if (RR == 2)
    Vx0=0.4; Ay0=1;
end
if (RR == 3)
    a1 = 5; a2 = 5;   % amplitude referentnog signala (1,0.5)
    w1 = 0.25; w2 = 1.25/2;     % frekvencije referentnog signala; (2,5)
end
%dPhi_d = 30; dTheta_d = 0; dPsi_d = 0;
Phi_d = 0; Theta_d = 0; Psi_d = 30;
x_d = 0; y_d = 0; z_d = 0;
dx_d = 0; dy_d = 0; dz_d = 0;
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

% Pole placement for X and Y trajectory tracking:
pol_1 = -3;
pol_2 = -3;
pol_3 = -4;
pol_4 = -5;

k_3 = -pol_1 - pol_2 - pol_3 - pol_4;
k_2 = pol_1*pol_2 + pol_2*pol_3 + pol_3*pol_4 + pol_1*pol_4 + pol_2*pol_4 + pol_3*pol_1;
k_1 = -pol_1*pol_2*pol_3 - pol_1*pol_2*pol_4 - pol_2*pol_3*pol_4 - pol_1*pol_3*pol_4;
k_0 = pol_1*pol_2*pol_3*pol_4;
%-------------------------------------------------------------------------%

% --- Fixed-step Runge-Kutta 4th order -----------------------------------%
if (WW == 1)
    tspan = [0 T]; Nstep = 20000; DeltaT = T/Nstep;
    [t, y] = rk4(@QuadroHB, tspan, xx0, DeltaT);
end
%-------------------------------------------------------------------------%

% --- ODE Runge-Kutta (variable step) ------------------------------------%
if (WW == 2)
    options = odeset('RelTol',1e-6,'AbsTol',1e-6);
    [t,y] = ode45('QuadroHB',[0 T],xx0,options);
end
%-------------------------------------------------------------------------%

fprintf('done! \n');
fprintf('Generating plots... ');
close all;


% === PLOTS ==============================================================%
% Set all plots position and size:
set(0, 'DefaultFigurePosition', [1367 -281 1920 973]); 

% --- Reference trajectories definitions for plots -----------------------%
if (RR == 1)
    x_d = zeros(size(t));
    y_d = zeros(size(t));
    z_d = zeros(size(t)); %needed to use 'end' below
    one_sec = (size(z_d, 1)-1)/T; % number of samples in one second
    
    % what ever T is, set step to start at 1 sec. and lower it to 0 at last quarter
    z_d = stepAmp*[zeros(size(t(1:one_sec))); ones(size(t(1:round(3*end/4)-one_sec))); zeros(size(t(1:round(1*end/4))))];
    dz_d = zeros(size(t));
end
if (RR == 2)
    x_d = -Ay0*0 + Ay0*cos(Vx0*t);
    y_d = Ay0*sin(Vx0*t);
    z_d = Vx0*t;
    dz_d = Vx0*ones(size(t));
end
if (RR == 3)
    x_d = zeros(size(t));
    y_d = zeros(size(t));
    z_d = a1*sin(w1*t) + a2*sin(w2*t);
    dz_d = w1*a1*cos(w1*t) + w2*a2*cos(w2*t);
end
%-------------------------------------------------------------------------%


% --- Disturbance definitions for plots ----------------------------------%
if (DD == 0)
    d_0 = 0*t;
end
if (DD == 1)
    d_0 = d0*exp(-Sg*(t-T/2).^2);
end
if (DD == 2)
    d_0 = d0*exp(-Sg*(t+5-1*T/4).^2) + d0*exp(-Sg*(t+5-2*T/4).^2) + d0*exp(-Sg*(t+5-3*T/4).^2) + d0*exp(-Sg*(t+5-4*T/4).^2);
end
if (DD == 3)
    d_0 = d0*exp(-Sg*(t+5-1*T/4).^2) - d0*exp(-Sg*(t+5-2*T/4).^2) + d0*exp(-Sg*(t+5-3*T/4).^2) - d0*exp(-Sg*(t+5-4*T/4).^2);
end
if (DD == 4)
    d_mx = 1.5 + 2.5*sin(4*t);
    d_my = 2.5 + 1.5*sin(3*t);
end
%-------------------------------------------------------------------------%

% Transfer actual references used in model
x_ref = diff(y(:, 54))./diff(t);
y_ref = diff(y(:, 55))./diff(t);
z_ref = diff(y(:, 56))./diff(t);

% Thrust force and torques
F =  diff(y(:,13))./diff(t);
T1 = diff(y(:,14))./diff(t);
T2 = diff(y(:,15))./diff(t);
T3 = diff(y(:,16))./diff(t);

de_z_est = diff(y(:,37))./diff(t);
de_y_est = diff(y(:,38))./diff(t);
de_x_est = diff(y(:,39))./diff(t);

de_z = diff(y(:,43))./diff(t);
de_y = diff(y(:,44))./diff(t);
de_x = diff(y(:,45))./diff(t);

Omega_orig1 = diff(y(:,46))./diff(t);
Omega_orig2 = diff(y(:,47))./diff(t);
Omega_orig3 = diff(y(:,48))./diff(t);
Omega_orig4 = diff(y(:,49))./diff(t);

Omega1 = diff(y(:,50))./diff(t);
Omega2 = diff(y(:,51))./diff(t);
Omega3 = diff(y(:,52))./diff(t);
Omega4 = diff(y(:,53))./diff(t);

td=t(1:(length(t)-1));

% --- MODEL SPECIFIC PLOTS -----------------------------------------------%

% --- MODEL 1 ------------------------------------------------------------%
if (QQ == 1)
    %Trajectories
    figure(1), set(gcf,'name','Trajectories','numbertitle','off')
    subplot(2,3,1), plot(t,y(:,1),'b', td, x_ref,'r:', 'linewidth',4), ylabel('x (m)','FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times'), grid on,
    subplot(2,3,2), plot(t,y(:,2),'b', td, y_ref,'r:', 'linewidth',4), ylabel('y (m)','FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times'), grid on,
    subplot(2,3,3), plot(t,y(:,3),'b', td, z_ref,'r:', 'linewidth',4), ylabel('z (m)','FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times'), grid on,%axis([0 10 0 1.2])
    subplot(2,3,4), plot(t,y(:,4),'b', 'linewidth',4), ylabel('\phi (rad)','FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times'), grid on,
    subplot(2,3,5), plot(t,y(:,5),'b', 'linewidth',4), ylabel('\theta (rad)','FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times'), grid on,
    subplot(2,3,6), plot(t,y(:,6),'b', 'linewidth',4), ylabel('\psi (rad)','FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times'), grid on,
    
    % Errors
    figure(2), set(gcf,'name','Errors','numbertitle','off')
    subplot(2,3,1), semilogy(td,abs(y(1:(end-1),1)-x_ref), 'b', 'linewidth',4), ylabel('|x-x_d| (m)','FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times'), grid on,
    subplot(2,3,2), semilogy(td,abs(y(1:(end-1),2)-y_ref), 'b', 'linewidth',4), ylabel('|y-y_d| (m)','FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times'), grid on,
    subplot(2,3,3), semilogy(td,abs(y(1:(end-1),3)-z_ref), 'b', 'linewidth',4), ylabel('|z-z_d| (m)','FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times'), grid on,
    
    % Force and torques
    figure(3), set(gcf,'name','Force and torques','numbertitle','off')
    subplot(2,2,1), plot(td,F,'b', 'linewidth',3), ylabel('F_z (N)','FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times'), grid on,%axis([0 5 9 15])
    subplot(2,2,2), plot(td,T1,'b', 'linewidth',3), ylabel('\tau_1 (Nm)','FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times'), grid on,%axis([0 5 -10 5])
    subplot(2,2,3), plot(td,T2,'b', 'linewidth',3), ylabel('\tau_2 (Nm)','FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times'), grid on,%axis([0 5 -1 5])
    subplot(2,2,4), plot(td,T3,'b', 'linewidth',3), ylabel('\tau_3 (Nm)','FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times'), grid on%axis([0 5 -1 1])
    
    % 3D trajectory
    figure(4), set(gcf,'name','3D trajectory','numbertitle','off')
    plot3(y(:,1),y(:,2),y(:,3), 'b', x_ref, y_ref ,z_ref, 'r:', 'linewidth',3),
    ylabel('x (m)','FontSize',16,'FontName','Times'), xlabel('y (m)','FontSize',16,'FontName','Times'), zlabel('z (m)','FontSize',16,'FontName','Times'),
    set(gca,'fontsize',14,'FontName','Times'), grid on, axis square
    
    % Model 1 rates
    figure(12), set(gcf,'name','Rates','numbertitle','off')
    subplot(2,3,1), plot(t, y(:,10), 'b', 'linewidth',4), ylabel('dPhi, dPhi_d (rad/s)','FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times'), grid on,
    subplot(2,3,2), plot(t, y(:,11), 'b', 'linewidth',4), ylabel('dTheta, dTheta_d (rad/s)','FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times'), grid on,
    subplot(2,3,3), plot(t, y(:,12), 'b', 'linewidth',4), ylabel('dPsi, dPsi_d (rad/s)','FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times'), grid on,
  
end
%-------------------------------------------------------------------------%

% --- MODEL 2 - MODEL 3 - MODEL 4 ----------------------------------------%
if (QQ == 2)||(QQ == 3)||(QQ == 4)    
    % Trajectories
    figure(1), set(gcf,'name','Trajectories','numbertitle','off')
    subplot(2,3,1), plot(t,y(:,1),'b', td, x_ref,'r:', 'linewidth',4), ylabel('x (m)','FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times'), grid on,
    subplot(2,3,2), plot(t,y(:,3),'b', td, y_ref,'r:', 'linewidth',4), ylabel('y (m)','FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times'), grid on,
    subplot(2,3,3), plot(t,y(:,5),'b', td, z_ref,'r:', 'linewidth',4), ylabel('z (m)','FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times'), grid on,
    subplot(2,3,4), plot(t,y(:,7),'b', 'linewidth',4), ylabel('\phi (rad)','FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times'), grid on,
    subplot(2,3,5), plot(t,y(:,9),'b', 'linewidth',4), ylabel('\theta (rad)','FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times'), grid on,
    subplot(2,3,6), plot(t,y(:,11),'b', 'linewidth',4), ylabel('\psi (rad)','FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times'), grid on,
    
    % Errors
    figure(2), set(gcf,'name','Errors','numbertitle','off')
    subplot(2,3,1), semilogy(td,abs(y(1:(end-1),1)-x_ref), 'b', 'linewidth',4), ylabel('|x-x_d| (m)','FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times'), grid on,
    subplot(2,3,2), semilogy(td,abs(y(1:(end-1),3)-y_ref), 'b', 'linewidth',4), ylabel('|y-y_d| (m)','FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times'), grid on,
    subplot(2,3,3), semilogy(td,abs(y(1:(end-1),5)-z_ref), 'b', 'linewidth',4), ylabel('|z-z_d| (m)','FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times'), grid on,
    
    % Force and torques
    figure(3), set(gcf,'name','Force and torques','numbertitle','off')
    subplot(2,2,1), plot(td,F,'b', 'linewidth',3), ylabel('F_z (N)','FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times'), grid on,%axis([0 5 9 15])
    subplot(2,2,2), plot(td,T1,'b', 'linewidth',3), ylabel('\tau_1 (Nm)','FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times'), grid on,%axis([0 5 -10 5])
    subplot(2,2,3), plot(td,T2,'b', 'linewidth',3), ylabel('\tau_2 (Nm)','FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times'), grid on,%axis([0 5 -1 5])
    subplot(2,2,4), plot(td,T3,'b', 'linewidth',3), ylabel('\tau_3 (Nm)','FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times'), grid on,%axis([0 5 -1 1])
    
    % 3D trajectory
    figure(4), set(gcf,'name','3D trajectory','numbertitle','off')
    plot3(y(:,1),y(:,3),y(:,5), 'b', x_ref, y_ref ,z_ref, 'r:', 'linewidth',3),
    ylabel('x (m)','FontSize',16,'FontName','Times'), xlabel('y (m)','FontSize',16,'FontName','Times'), zlabel('z (m)','FontSize',16,'FontName','Times'),
    set(gca,'fontsize',14,'FontName','Times'), grid on, axis square
    
    dPhi_d = diff(y(:,57))./diff(t);
    dTheta_d = diff(y(:,58))./diff(t);
    dPsi_d = diff(y(:,59))./diff(t);
    
    % Model 1 rates
    figure(12), set(gcf,'name','Rates','numbertitle','off')
    subplot(2,3,1), plot(t, y(:,8), 'b', td, dPhi_d, 'r:', 'linewidth',4), ylabel('dPhi, dPhi_d (rad/s)','FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times'), grid on,
    subplot(2,3,2), plot(t, y(:,10), 'b', td, dTheta_d, 'r:', 'linewidth',4), ylabel('dTheta, dTheta_d (rad/s)','FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times'), grid on,
    subplot(2,3,3), plot(t, y(:,12), 'b', td, dPsi_d, 'r:', 'linewidth',4), ylabel('dPsi, dPsi_d (rad/s)','FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times'), grid on,
      
end % MODEL SPECIFIC
%-------------------------------------------------------------------------%

% --- GENERAL PLOTS ------------------------------------------------------%
% % Estimated error derivatives
% if (EE ~= 0)
%     figure(5), set(gcf,'name','Estimated error derivatives','numbertitle','off')
%     subplot(2,3,1), plot(td, de_x_est,'b-', td, de_x, 'r:', 'linewidth',4), ylabel('e_x, e_{x,est}','FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times'), grid on,
%     legend('de_{x, est}', 'de_x');
%     subplot(2,3,4), semilogy(td, abs(de_x - de_x_est), '-b', 'linewidth',4), ylabel('|de_x - de_{x,est}|','FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times'), grid on
%     
%     subplot(2,3,2), plot(td, de_y_est,'b-', td, de_y, 'r:', 'linewidth',4), ylabel('e_y, e_{y,est}','FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times'), grid on,
%     legend('de_{y, est}', 'de_y');
%     subplot(2,3,5), semilogy(td, abs(de_y - de_y_est), '-b', 'linewidth',4), ylabel('|de_y - de_{y,est}|','FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times'), grid on
%     
%     subplot(2,3,3), plot(td, de_z_est,'b-', td, de_z, 'r:', 'linewidth',4), ylabel('e_z, e_{z,est}','FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times'), grid on,
%     legend('de_{z, est}', 'de_z');
%     subplot(2,3,6), semilogy(td, abs(de_z - de_z_est), '-b', 'linewidth',4), ylabel('|de_z - de_{z,est}|','FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times'), grid on
% end
% 
% % Filtered reference
% if (SF ~= 0)
%     figure(6), set(gcf,'name','Filtered X direction reference','numbertitle','off')
%     subplot(3,1,1), plot(t, y(:, 27), 'b-', t, x_d, 'r:', 'Linewidth', 4), ylabel('x_d (m)','FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times'), grid on,
%     legend('1st order smoothing filter', 'Original X reference');
%     subplot(3,1,2), plot(t, y(:, 30), 'b-', t, x_d, 'r:', 'Linewidth', 4), ylabel('x_d (m)','FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times'), grid on,
%     legend('2nd order smoothing filter', 'Original X reference');
%     subplot(3,1,3), plot(t, y(:, 33), 'b-', t, x_d, 'r:', 'Linewidth', 4), ylabel('x_d (m)','FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times'), grid on,
%     legend('3rd order smoothing filter', 'Original X reference');
%     
%     figure(7), set(gcf,'name','Filtered Y direction reference','numbertitle','off')
%     subplot(3,1,1), plot(t, y(:, 26), 'b-', t, y_d, 'r:', 'Linewidth', 4), ylabel('y_d (m)','FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times'), grid on,
%     legend('1st order smoothing filter', 'Original Y reference');
%     subplot(3,1,2), plot(t, y(:, 29), 'b-', t, y_d, 'r:', 'Linewidth', 4), ylabel('y_d (m)','FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times'), grid on,
%     legend('2nd order smoothing filter', 'Original Y reference');
%     subplot(3,1,3), plot(t, y(:, 32), 'b-', t, y_d, 'r:', 'Linewidth', 4), ylabel('y_d (m)','FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times'), grid on,
%     legend('3rd order smoothing filter', 'Original Y reference');
%     
%     figure(8), set(gcf,'name','Filtered Z direction reference','numbertitle','off')
%     subplot(3,1,1), plot(t, y(:, 25), 'b-', t, z_d, 'r:', 'Linewidth', 4), ylabel('z_d (m)','FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times'), grid on,
%     legend('1st order smoothing filter', 'Original Z reference');
%     subplot(3,1,2), plot(t, y(:, 28), 'b-', t, z_d, 'r:', 'Linewidth', 4), ylabel('z_d (m)','FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times'), grid on,
%     legend('2nd order smoothing filter', 'Original Z reference');
%     subplot(3,1,3), plot(t, y(:, 31), 'b-', t, z_d, 'r:', 'Linewidth', 4), ylabel('z_d (m)','FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times'), grid on,
%     legend('3rd order smoothing filter', 'Original Z reference');
% end
% 
% % Disturbance shape
% if (DD == 4)
%     figure(9), set(gcf,'name','Disturbance shape','numbertitle','off')
%     subplot(2,1,1), plot(t, d_mx, 'b-', 'Linewidth', 4), ylabel('Wind gust in X direction', 'FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times'), grid on;
%     subplot(2,1,2), plot(t, d_my, 'b-', 'Linewidth', 4), ylabel('Wind gust in Y direction', 'FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times'), grid on;
% else if (DD ~= 0)
%     figure(9), set(gcf,'name','Disturbance shape','numbertitle','off')
%     plot(t, d_0, 'b-', 'Linewidth', 4), ylabel('Wind gust', 'FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times'), grid on;
%     end
% end

% % Requested actuator angular velocities
% figure(10), set(gcf,'name','Requested actuator angular velocities','numbertitle','off')
% subplot(2,2,1), plot(td, Omega_orig1, 'b-', 'LineWidth', 4), ylabel('Actual angular speed [rad/s]','FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times'), grid on,
% legend('Actuator 1');
% subplot(2,2,2), plot(td, Omega_orig2, 'b-', 'LineWidth', 4), ylabel('Actual angular speed [rad/s]','FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times'), grid on,
% legend('Actuator 2');
% subplot(2,2,3), plot(td, Omega_orig3, 'b-', 'LineWidth', 4), ylabel('Actual angular speed [rad/s]','FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times'), grid on,
% legend('Actuator 3');
% subplot(2,2,4), plot(td, Omega_orig4, 'b-', 'LineWidth', 4), ylabel('Actual angular speed [rad/s]','FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times'), grid on,
% legend('Actuator 4');

% Saturated actuator angular velocities
figure(11), set(gcf,'name','Saturated actuator angular velocities','numbertitle','off')
subplot(2,2,1), plot(td, Omega1, 'b-', 'LineWidth', 4), ylabel('Saturated angular speed [rad/s]','FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times'), grid on,
legend('Actuator 1');
subplot(2,2,2), plot(td, Omega2, 'b-', 'LineWidth', 4), ylabel('Saturated angular speed [rad/s]','FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times'), grid on,
legend('Actuator 2');
subplot(2,2,3), plot(td, Omega3, 'b-', 'LineWidth', 4), ylabel('Saturated angular speed [rad/s]','FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times'), grid on,
legend('Actuator 3');
subplot(2,2,4), plot(td, Omega4, 'b-', 'LineWidth', 4), ylabel('Saturated angular speed [rad/s]','FontSize',16,'FontName','Times'), xlabel('time (sec)','FontSize',16,'FontName','Times'), set(gca,'fontsize',14,'FontName','Times'), grid on,
legend('Actuator 4');

fprintf('done!\n');
%=========================================================================%