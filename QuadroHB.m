function dy = QuadroHB(t,y)

global N T QQ YY DD RR SF EE SAT
global grav mm Ixx Iyy Izz I_B d0 Sg Vx0 Ay0 a1 a2 w1 w2 stepAmp
global k_P k_D kk_P kk_D kk_I k_3 k_2 k_1 k_0 Ke_lin Ke_st Ksf rho u kg
global E_B inv_E_B AngVel_limit dX_d dY_d dZ_d

dy = zeros(N, 1);

% --- STATE VARIABLES - MODEL 1 ------------------------------------------%
if (QQ == 1)    
    X=y(1); dX=y(7); % #TODO: Fix MODEL 1
    Y=y(2); dY=y(8);
    Z=y(3); dZ=y(9);
    
    Phi=y(4); dPhi=y(10);
    Theta=y(5); dTheta=y(11);
    Psi=y(6); dPsi=y(12);
    
    Position = [X; Y; Z;];
    Angle = [Phi; Theta; Psi];
    Velocity_Lin_B = y(7:9);
    Velocity_Ang_B = y(10:12);
    Velocity_B = [Velocity_Lin_B; Velocity_Ang_B];
end
%-------------------------------------------------------------------------%

% --- STATE VARIABLES - MODEL 2 - MODEL 3 - MODEL 4 ----------------------%
if (QQ == 2)||(QQ == 3)||(QQ == 4)
    X=y(1); dX=y(2);
    Y=y(3); dY=y(4);
    Z=y(5); dZ=y(6);
    
    Phi=y(7); dPhi=y(8);
    Theta=y(9); dTheta=y(10);
    Psi=y(11); dPsi=y(12);
end
%-------------------------------------------------------------------------%


% --- Reference trajectory parameters ------------------------------------%
if (RR == 0)
    x_d = 0; y_d = 0; z_d = 0;
    dx_d = 0; dy_d = 0; dz_d = 0;
end

if (RR == 1)
    % what ever T is, set step to start at 1 sec. and lower it to 0 at last quarter
    if(t<1)
        st = 0;
    else if (t>3*T/4)
            st = 0;
        else
            st = 1;
        end
    end
%     % X step
%     x_d = st * stepAmp; dx_d=0; ddx_d=0; d3x_d=0; d4x_d=0;
%     y_d = 0; dy_d=0; ddy_d=0; d3y_d=0; d4y_d=0;    
%     z_d = 0; dz_d=0; ddz_d=0; d3z_d=0; d4z_d=0;
    
    % Y step
%     x_d = 0; dx_d=0; ddx_d=0; d3x_d=0; d4x_d=0;
%     y_d = st * stepAmp; dy_d=0; ddy_d=0; d3y_d=0; d4y_d=0;    
%     z_d = 0; dz_d=0; ddz_d=0; d3z_d=0; d4z_d=0;
    
    % Z step
    x_d = 0; dx_d=0; ddx_d=0; d3x_d=0; d4x_d=0;
    y_d = 0; dy_d=0; ddy_d=0; d3y_d=0; d4y_d=0;
    z_d = st * stepAmp; dz_d=0; ddz_d=0; d3z_d=0; d4z_d=0;
end

if (RR == 2)
    x_d = -Ay0*0 + Ay0*cos(Vx0*t);
    y_d = Ay0*sin(Vx0*t);
    z_d = Vx0*t;
    
    dx_d = -Ay0*Vx0*sin(Vx0*t); ddx_d = -Ay0*Vx0^2*cos(Vx0*t); d3x_d = Ay0*Vx0^3*sin(Vx0*t); d4x_d = Ay0*Vx0^4*cos(Vx0*t);
    dy_d = Ay0*Vx0*cos(Vx0*t); ddy_d = -Ay0*Vx0^2*sin(Vx0*t); d3y_d = -Ay0*Vx0^3*cos(Vx0*t); d4y_d = Ay0*Vx0^4*sin(Vx0*t);
    dz_d = Vx0; ddz_d = 0; d3z_d = 0; d4z_d = 0;
    
end

if (RR == 3)
    x_d = 0; dx_d=0; ddx_d=0; d3x_d=0; d4x_d=0;
    y_d = 0; dy_d=0; ddy_d=0; d3y_d=0; d4y_d=0;
    
    z_d = a1*sin(w1*t) + a2*sin(w2*t);
    dz_d = w1*a1*cos(w1*t) + w2*a2*cos(w2*t);
    ddz_d = -w1^2*a1*sin(w1*t) - w2^2*a2*sin(w2*t);
end
%-------------------------------------------------------------------------%

% --- Reference smoothing filters ----------------------------------------%
if (SF == 0) % References w/o smoothing filter
    z_ref = z_d;
    dz_ref = dz_d;
    
    y_ref = y_d;
    dy_ref = dy_d;
    
    x_ref = x_d;
    dx_ref = dx_d;
end
if (SF == 1)||(SF == 2)
    % First derivative of ref. = dy(18) / 1st order smoothed ref. = y(18)
    dz_df = -Ksf*(y(25) - z_d);
    dy_df = -Ksf*(y(26) - y_d);
    dx_df = -Ksf*(y(27) - x_d);
    
    % Second derivative of ref. = dy(19) / 2nd order smoothed ref. = y(19)
    ddz_df = -Ksf*(y(28) - y(25));
    ddy_df = -Ksf*(y(29) - y(26));
    ddx_df = -Ksf*(y(30) - y(27));
    
    d3z_d = -Ksf*(y(31) - y(28));
    d3y_d = -Ksf*(y(32) - y(29));
    d3x_d = -Ksf*(y(33) - y(30));
    
    d4z_d = -Ksf*(y(34) - y(31));
    d4y_d = -Ksf*(y(35) - y(32));
    d4x_d = -Ksf*(y(36) - y(33));
    
    if (SF == 1) % 1st order smoothing filter used
        z_ref = y(25);
        y_ref = y(26);
        x_ref = y(27);
        dz_ref = dz_df;
        dy_ref = dy_df;
        dx_ref = dx_df;
    end
    
    if (SF == 2) % 2nd order smoothing filter used
        z_ref = y(28);
        y_ref = y(29);
        x_ref = y(30);
        dz_ref = ddz_df;
        dy_ref = ddy_df;
        dx_ref = ddx_df;
    end
end

if (SF == 3) % Nonlinear saturated smoothing filter used
    dz_df = -rho*tanh(u*(y(25) - z_d));
    dy_df = -rho*tanh(u*(y(26) - y_d));
    dx_df = -rho*tanh(u*(y(27) - x_d));
    
    ddz_df = -rho*tanh(u*(y(28) - y(25)));
    ddy_df = -rho*tanh(u*(y(29) - y(26)));
    ddx_df = -rho*tanh(u*(y(30) - y(27)));
    
    d3z_d = -rho*tanh(u*(y(31) - y(28)));
    d3y_d = -rho*tanh(u*(y(32) - y(29)));
    d3x_d = -rho*tanh(u*(y(33) - y(30)));
    
    d4z_d = -rho*tanh(u*(y(34) - y(31)));
    d4y_d = -rho*tanh(u*(y(35) - y(32)));
    d4x_d = -rho*tanh(u*(y(36) - y(33)));
    
    z_ref = y(25);
    y_ref = y(26);
    x_ref = y(27);
    dz_ref = dz_df;
    dy_ref = dy_df;
    dx_ref = dx_df;
end
%-------------------------------------------------------------------------%


% --- Error variables ----------------------------------------------------%
e_x = X-x_ref; de_x = dX-dx_ref;
e_y = Y-y_ref; de_y = dY-dy_ref;
e_z = Z-z_ref; de_z = dZ-dz_ref;
%-------------------------------------------------------------------------%


% --- Error derivative estimator -----------------------------------------%
if (EE == 0) % assume state variables are perfectly measured
    de_z_est = de_z;
    de_y_est = de_y;
    de_x_est = de_x;
end

if (EE == 1) % estimate error derivatives with linear estimator
    de_z_est = -Ke_lin*(y(37) - e_z);
    de_y_est = -Ke_lin*(y(38) - e_y);
    de_x_est = -Ke_lin*(y(39) - e_x);
end

if (EE == 2) % estimate error derivatives with super twisting estimator
    de_z_est = -Ke_st*sqrt(abs(y(37)-e_z))*sign(y(37)-e_z) + y(40);
    de_y_est = -Ke_st*sqrt(abs(y(38)-e_y))*sign(y(38)-e_y) + y(41);
    de_x_est = -Ke_st*sqrt(abs(y(39)-e_x))*sign(y(39)-e_x) + y(42);
end
%-------------------------------------------------------------------------%


% --- Quadrotor parameters (nominal) -------------------------------------%
% Parameters m, Ix, Iy, Iz are used in controllers (assumed values)
% Parameters mm, Ixx, Iyy, Izz are actual quadrotor values (physical model)
m = 1.0*mm;
Ix = 1.0*Ixx;
Iy = 1.0*Iyy;
Iz = 1.0*Izz;
%-------------------------------------------------------------------------%

% === CONTROLLERS ========================================================%
% --- PD controller ------------------------------------------------------%
if (YY == 1)    
    % U_0 = m*(grav + k_D*dZ + k_P*e_z); % z velocity is measured (dZ known)
    % U_0 = m*(grav + k_D*de_z_est + k_P*e_z); % velocity is not measured, derivatives are estimated
    U_0 = max((m*(grav + k_D*de_z_est + k_P*e_z)), 0); % limit to positive numbers only
    
    U_1 = -k_D*dPhi - k_P*Phi;
    U_2 = -k_D*dTheta - k_P*Theta;
    U_3 = -k_D*dPsi - k_P*Psi;
end
%-------------------------------------------------------------------------%

% --- PID controller -----------------------------------------------------%
if (YY == 2)    
    % U_0 = kk_D*de_z_est + kk_P*e_z + kk_I*y(17); % PID control
    % U_0 = m*grav +kk_D*de_z_est + kk_P*e_z + kk_I*y(17); % PID control w/ gravity compensation
    U_0 = max((m*grav +kk_D*de_z_est + kk_P*e_z + kk_I*y(17)), 0); % limit to positive numbers only
    
    U_1 = -kk_D*dPhi - kk_P*Phi - kk_I*y(18);
    U_2 = -kk_D*dTheta - kk_P*Theta - kk_I*y(19);
    U_3 = -kk_D*dPsi - kk_P*Psi - kk_I*y(20);
end
%-------------------------------------------------------------------------%

% --- Sliding mode 1st order (sign) --------------------------------------%
if (YY == 3)    
    p = 1; eps = 0.01;
    Uz = 20; % 4 values tested: 20, 50, 100, 150
    % s0 = de_z + p*e_z;
    s0 = de_z_est + p*e_z;
    
    % U_0 = Uz*sign(s0);
    % U_0 = m*Uz*sign(s0);
    % U_0 = m*(grav + Uz*sign(s0));
    % U_0 = m*(grav + Uz*s0 - Uz*sign(s0));
    
    % U_0 = m*(grav + Uz*s0 + Uz*( s0 / (abs(s0) + eps) ) );
    
    U_0 = max(m*(grav + Uz*s0 + Uz*sign(s0)), 0); % limit to positive numbers only
    
    % s0 = de_z_est + (k_P/k_D)*e_z;
    % U_0 = m*(grav + k_D*s0 + k_D*sign(s0));
    % U_0 = m*(grav + k_D*s0 + k_D*( s0 / (abs(s0) + eps) ) );
    % U_0 = max((m*(grav + k_D*s0 + k_D*( s0 / (abs(s0) + eps) ) )), 0); % limit to positive numbers only
    
    U_1 = -kk_D*dPhi - kk_P*Phi - kk_I*y(18);
    U_2 = -kk_D*dTheta - kk_P*Theta - kk_I*y(19);
    U_3 = -kk_D*dPsi - kk_P*Psi - kk_I*y(20);
end
%-------------------------------------------------------------------------%

% --- Super-twisting controller ------------------------------------------%
if (YY == 4)    
    p = 3; Uz = 20; % 20, 50, 80, 100
    
    s0 = de_z_est + p*e_z;
    % s0 = de_z_est + (k_P/k_D)*e_z;
    
    ST_0 = -Uz*sqrt(abs(s0))*sign(s0) + y(21);
    % ST_0 = -sqrt(Uz)*sqrt(abs(s0))*sign(s0) + y(21);
    
    % U_0 = ST_0; % add pure super-twisting
    % U_0 = m*ST_0;
    % U_0 = m*(grav + ST_0);
    % U_0 = m*(grav - Uz*s0 + ST_0);
    % U_0 = m*(grav - k_D*s0) + ST_0;
    
    %k_m = 4;
    %U_0 = m*grav + k_m*tanh( -(m/k_m)*Uz*s0 + (1/k_m)*ST_0);
    
    U_0 = max((m*grav + m*Uz*s0 - ST_0), 0); % limit to positive numbers only
    
    U_1 = -kk_D*dPhi - kk_P*Phi - kk_I*y(18);
    U_2 = -kk_D*dTheta - kk_P*Theta - kk_I*y(19);
    U_3 = -kk_D*dPsi - kk_P*Psi - kk_I*y(20);
end
%-------------------------------------------------------------------------%


% --- Trajectory tracking control law ------------------------------------%
if (YY == 5)    
    % U_0 = kk_D*de_z_est + kk_P*e_z + kk_I*y(17); % PID control
    % U_0 = m*grav +kk_D*de_z_est + kk_P*e_z + kk_I*y(17); % PID control w/ gravity compensation
    U_0 = max(-m*(-grav + ddz_d -kk_D*de_z_est - kk_P*e_z - kk_I*y(17)), 0); % limit to positive numbers only
    
    dde_x = (-grav/m)*Theta - ddx_d;
    d3e_x = (-grav/m)*dTheta - d3x_d;
    
    dde_y = (grav/m)*Phi - ddy_d;
    d3e_y = (grav/m)*dPhi - d3y_d;
    
    U_1 = ((m*Ix)/grav)*(d4y_d - k_3*d3e_y - k_2*dde_y - k_1*de_y_est - k_0*e_y);
    U_2 = ((-m*Iy)/grav)*(d4x_d - k_3*d3e_x - k_2*dde_x - k_1*de_x_est - k_0*e_x);
    U_3 = Iz*(-kk_D*dPsi - kk_P*Psi - kk_I*y(20));
end
%-------------------------------------------------------------------------%

% --- 1-SM Trajectory tracking control law -------------------------------%
if (YY == 6)    
    dde_x = -Theta*grav - ddx_d;
    d3e_x = -dTheta*grav - d3x_d;
    
    dde_y = Phi*grav - ddy_d;
    d3e_y = dPhi*grav - d3y_d;
    
    % Sliding surfaces
    %lambda = 3; p = 3;
    lambda = 5; p = 5;
    lambdaz = 1; pz = 1;
    alpha_z0 = pz;
    alpha_x0 = p^3; alpha_x1 = 3*p^2; alpha_x2 = 3*p;
    alpha_y0 = p^3; alpha_y1 = 3*p^2; alpha_y2 = 3*p;
    alpha_psi0 = pz;
    
    Uz = 0; Ux = 4; Uy = 4; Upsi = 0;
    % Z
    s0 = de_z_est + alpha_z0*e_z;
    k_z1 = alpha_z0 + lambdaz;
    k_z0 = alpha_z0 * lambdaz;
    
    %X
    s2 = d3e_x + alpha_x2*dde_x + alpha_x1*de_x_est + alpha_x0*e_x;
    k_x3 = alpha_x2 + lambda;
    k_x2 = alpha_x1 + lambda*alpha_x2;
    k_x1 = alpha_x0 + lambda*alpha_x1;
    k_x0 = lambda * alpha_x0;
    
    %Y
    s1 = d3e_y + alpha_y2*dde_y + alpha_y1*de_y_est + alpha_y0*e_y;
    k_y3 = alpha_y2 + lambda;
    k_y2 = alpha_y1 + lambda*alpha_y2;
    k_y1 = alpha_y0 + lambda*alpha_y1;
    k_y0 = lambda * alpha_y0;
    
    % PSI
    s3 = dPsi + alpha_psi0*Psi;
    k_psi1 = alpha_psi0 + lambdaz;
    k_psi0 = alpha_psi0 * lambdaz;
    
    
    % 1-SM controllers:
    % Signum
    SM_0 = -Uz*sign(s0);
    SM_1 = -Uy*sign(s1);
    SM_2 = -Ux*sign(s2);
    SM_PSI = -Upsi*sign(s3);
    
    % Smooth variant tanh
    % K = 20;
    % SM_0 = -Uz*tanh(K*s0);
    % SM_1 = -Uy*tanh(K*s1);
    % SM_2 = -Ux*tanh(K*s2);
    % SM_PSI = -Upsi*tanh(K*s3);
    
    % Control laws
    U_0 = max((-m*(ddz_d - grav -k_z1*de_z_est - k_z0*e_z) - SM_0), 0);
    U_1 = (Ix/grav)*(d4y_d - k_y3*d3e_y - k_y2*dde_y - k_y1*de_y_est - k_y0*e_y) + SM_1;
    U_2 = (-Iy/grav)*(d4x_d - k_x3*d3e_x - k_x2*dde_x - k_x1*de_x_est - k_x0*e_x) - SM_2;
    U_3 = Iz*(-k_psi1*dPsi - k_psi0*Psi) + SM_PSI;
end
%-------------------------------------------------------------------------%

% --- Super-twisting Trajectory tracking control law ---------------------%
if (YY == 7)    
    dde_x = -Theta*grav - ddx_d;
    d3e_x = -dTheta*grav - d3x_d;
    
    dde_y = Phi*grav - ddy_d;
    d3e_y = dPhi*grav - d3y_d;
    
    % Sliding surfaces
    %lambda = 3; p = 3;
    lambda = 5; p = 5;
    lambdaz = 1; pz = 1;
    alpha_z0 = pz;
    alpha_x0 = p^3; alpha_x1 = 3*p^2; alpha_x2 = 3*p;
    alpha_y0 = p^3; alpha_y1 = 3*p^2; alpha_y2 = 3*p;
    alpha_psi0 = pz;
    
    Uz = 0; Ux = 0.6; Uy = 0.6; Upsi = 0;
    
    % Z
    s0 = de_z_est + alpha_z0*e_z;
    k_z1 = alpha_z0 + lambdaz;
    k_z0 = alpha_z0*lambdaz;
    
    %X
    s2 = d3e_x + alpha_x2*dde_x + alpha_x1*de_x_est + alpha_x0*e_x;
    k_x3 = alpha_x2 + lambda;
    k_x2 = alpha_x1 + lambda*alpha_x2;
    k_x1 = alpha_x0 + lambda*alpha_x1;
    k_x0 = lambda * alpha_x0;
    
    %Y
    s1 = d3e_y + alpha_y2*dde_y + alpha_y1*de_y_est + alpha_y0*e_y;
    k_y3 = alpha_y2 + lambda;
    k_y2 = alpha_y1 + lambda*alpha_y2;
    k_y1 = alpha_y0 + lambda*alpha_y1;
    k_y0 = lambda * alpha_y0;
    
    % PSI
    s3 = dPsi + alpha_psi0*Psi;
    k_psi1 = alpha_psi0 + lambdaz;
    k_psi0 = alpha_psi0 * lambdaz;
    
    % Super-twisting controllers:
    ST_0 = -Uz*sqrt(abs(s0))*sign(s0) + y(21);
    ST_1 = -Uy*sqrt(abs(s1))*sign(s1) + y(22);
    ST_2 = -Ux*sqrt(abs(s2))*sign(s2) + y(23);
    ST_PSI = -Upsi*sqrt(abs(s3))*sign(s3) + y(24);
    
    % Control laws
    U_0 = max((-m*(ddz_d - grav -k_z1*de_z_est - k_z0*e_z) - ST_0), 0);
    U_1 = (Ix/grav)*(d4y_d - k_y3*d3e_y - k_y2*dde_y - k_y1*de_y_est - k_y0*e_y) + ST_1;
    U_2 = (-Iy/grav)*(d4x_d - k_x3*d3e_x - k_x2*dde_x - k_x1*de_x_est - k_x0*e_x) - ST_2;
    U_3 = Iz*(-k_psi1*dPsi - k_psi0*Psi) + ST_PSI;
end
%-------------------------------------------------------------------------%

if (YY == 8)
    % Position P controller gains
%     pos_xy_p = 0.95; % PX4
%     pos_z_p = 1;
    
    pos_xy_p = -1.4; % mine
    pos_z_p = -2.5;


    % Linear velocity PID controller gains
%     vel_xy_p = 0.09;
%     vel_xy_i = 0.02;
%     vel_xy_d = 0.01;
%     vel_z_p = 0.2;
%     vel_z_i = 0.02;
%     vel_z_d = 0;
    
    vel_xy_p = 0.2;
    vel_xy_i = 0;
    vel_xy_d = 0;
    vel_z_p = 140;
    vel_z_i = 60;
    vel_z_d = 60;


    % Angle P controller gains
%     att_phi_p = 6.5;
%     att_theta_p = 6.5;
%     att_psi_p = 2.8;
    
    att_phi_p = 4;
    att_theta_p = 4;
    att_psi_p = 0.7;
       

    % Angular velocity PID controller gains
%     rate_rollrate_p = 0.15;
%     rate_rollrate_i = 0.05;
%     rate_rollrate_d = 0.003;
%     rate_pitchrate_p = 0.15;
%     rate_pitchrate_i = 0.05;
%     rate_pitchrate_d = 0.003;
%     rate_yawrate_p = 0.2;
%     rate_yawrate_i = 0.1;
%     rate_yawrate_d = 0;
    
    rate_rollrate_p = 70.5;
    rate_rollrate_i = 0;
    rate_rollrate_d = 0;
    rate_pitchrate_p = 70.5;
    rate_pitchrate_i = 0;
    rate_pitchrate_d = 0;
    rate_yawrate_p = 85.5;
    rate_yawrate_i = 1;
    rate_yawrate_d = 0;

    
    % --- POSITION P CONTROLLERS --- %
    vel_x_sp = pos_xy_p * e_x; % _sp == setpoint
    vel_y_sp = pos_xy_p * e_y;
    vel_z_sp = pos_z_p * e_z;
    
    
    % --- LINEAR VELOCITY PID CONTROLLERS --- %
    % Errors
    e_vel_x = - vel_x_sp + dX;
    e_vel_y = vel_y_sp - dY;
    e_vel_z = vel_z_sp - dZ;
    % Derivatives
    de_vel_x_est = -Ke_lin*(y(72) - e_vel_x);
    de_vel_y_est = -Ke_lin*(y(73) - e_vel_y);
    de_vel_z_est = -Ke_lin*(y(74) - e_vel_z);
    % Linear velocity PID 
    Theta_sp = vel_xy_p * e_vel_x + vel_xy_i * y(75) + vel_xy_d * de_vel_x_est;
    Phi_sp = vel_xy_p * e_vel_y + vel_xy_i * y(76) + vel_xy_d * de_vel_y_est;
    Psi_sp = 0;
    
    vel_z = vel_z_p * e_vel_z + vel_z_i * y(77) + vel_z_d * de_vel_z_est;
    
    throttle = 966 - vel_z;
    
    
    % --- ANGLE P CONTROLLERS --- %    
    % Angle errors
    e_Phi = Phi_sp - Phi;
    e_Theta = Theta_sp - Theta;
    e_Psi = Psi_sp - Psi;           
    % Angle P controllers (stabilized)
    rollrate_sp = att_phi_p * e_Phi;
    pitchrate_sp = att_theta_p * e_Theta;
    yawrate_sp = att_psi_p * e_Psi;
    
    
    % --- RATE PID CONTROLLERS --- %
    % Angular velocity errors
    e_dPhi = dPhi - rollrate_sp;
    e_dTheta = dTheta - pitchrate_sp;
    e_dPsi = dPsi - yawrate_sp;    
    % Derivatives
    de_dPhi_est = -Ke_lin*(y(60) - e_dPhi);
    de_dTheta_est = -Ke_lin*(y(61) - e_dTheta);
    de_dPsi_est = -Ke_lin*(y(62) - e_dPsi);        
    % Angular velocity PID controllers (rates)
    roll_output = rate_rollrate_p * e_dPhi + rate_rollrate_i * y(63) + rate_rollrate_d * de_dPhi_est;
    pitch_output = rate_pitchrate_p * e_dTheta + rate_pitchrate_i * y(64) + rate_pitchrate_d * de_dTheta_est;
    yaw_output = rate_yawrate_p * e_dPsi + rate_yawrate_i * y(65) + rate_yawrate_d * de_dPsi_est;
    
    %throttle = 1050;
    
    Omega = [ throttle - roll_output + pitch_output - yaw_output;
              throttle + roll_output - pitch_output - yaw_output;
              throttle + roll_output + pitch_output + yaw_output;
              throttle - roll_output - pitch_output + yaw_output; ];
       
    U = E_B * Omega.^2;
    U_0 = U(1);
    U_1 = U(2);
    U_2 = U(3);
    U_3 = U(4);   
             
end
%=========================================================================%

% --- Actuator saturation ------------------------------------------------%
Omega_orig = real(sqrt(inv_E_B*[U_0 U_1 U_2 U_3]'));

% if (max(Omega_orig) > AngVel_limit)
%     scale_factor = AngVel_limit/max(Omega_orig);
%     Omega = Omega_orig .* scale_factor;
% else
%     Omega = Omega_orig;
% end
% %Omega = AngVel_limit.*tanh(Omega./AngVel_limit); % old version
% 
% FF = E_B * Omega.^2;
%-------------------------------------------------------------------------%

% --- Disturbances (wind gust) -------------------------------------------%
if (DD == 0)
    d_0=0; d_1=0; d_2=0; d_3=0;
end

if (DD == 1)
    d_0 = 0*d0*exp(-Sg*(t-T/2)^2);
    d_1 = 1*d0*exp(-Sg*(t-T/2)^2);
    d_2 = 0*d0*exp(-Sg*(t-T/2)^2);
    d_3 = 0*d0*exp(-Sg*(t-T/2)^2);
end

if (DD == 2)
    d_0 = 0*(d0*exp(-Sg*(t+5-1*T/4).^2) + d0*exp(-Sg*(t+5-2*T/4).^2) + d0*exp(-Sg*(t+5-3*T/4).^2) + d0*exp(-Sg*(t+5-4*T/4).^2));
    d_1 = 1*(d0*exp(-Sg*(t+5-1*T/4).^2) + d0*exp(-Sg*(t+5-2*T/4).^2) + d0*exp(-Sg*(t+5-3*T/4).^2) + d0*exp(-Sg*(t+5-4*T/4).^2));
    d_2 = 0*(d0*exp(-Sg*(t+5-1*T/4).^2) + d0*exp(-Sg*(t+5-2*T/4).^2) + d0*exp(-Sg*(t+5-3*T/4).^2) + d0*exp(-Sg*(t+5-4*T/4).^2));
    d_3 = 0*(d0*exp(-Sg*(t+5-1*T/4).^2) + d0*exp(-Sg*(t+5-2*T/4).^2) + d0*exp(-Sg*(t+5-3*T/4).^2) + d0*exp(-Sg*(t+5-4*T/4).^2));
end

if (DD == 3)
    d_0 = 0*(d0*exp(-Sg*(t+5-1*T/4).^2) - d0*exp(-Sg*(t+5-2*T/4).^2) + d0*exp(-Sg*(t+5-3*T/4).^2) - d0*exp(-Sg*(t+5-4*T/4).^2));
    d_1 = 0*(d0*exp(-Sg*(t+5-1*T/4).^2) - d0*exp(-Sg*(t+5-2*T/4).^2) + d0*exp(-Sg*(t+5-3*T/4).^2) - d0*exp(-Sg*(t+5-4*T/4).^2));
    d_2 = 1*(d0*exp(-Sg*(t+5-1*T/4).^2) - d0*exp(-Sg*(t+5-2*T/4).^2) + d0*exp(-Sg*(t+5-3*T/4).^2) - d0*exp(-Sg*(t+5-4*T/4).^2));
    d_3 = 0*(d0*exp(-Sg*(t+5-1*T/4).^2) - d0*exp(-Sg*(t+5-2*T/4).^2) + d0*exp(-Sg*(t+5-3*T/4).^2) - d0*exp(-Sg*(t+5-4*T/4).^2));
end

if (DD == 4)
    d_0 = 0;
    d_1 = 2.5 + 1.5*sin(3*t);
    d_2 = 1.5 + 2.5*sin(4*t);
    d_3 = 0;
end
%-------------------------------------------------------------------------%


% --- Motor saturation ---------------------------------------------------%
%F = kg*tanh((U_0 + d_0)/kg);

if (SAT) % With saturation
    F = FF(1) + d_0;
    T_1 = FF(2) + d_1;
    T_2 = FF(3) + d_2;
    T_3 = FF(4) + d_3;
else % Without saturation
    F = U_0 + d_0;
    T_1 = U_1 + d_1;
    T_2 = U_2 + d_2;
    T_3 = U_3 + d_3;
end
%-------------------------------------------------------------------------%

% --- MODEL 1 ------------------------------------------------------------%
if (QQ == 1)    
    % Rotation matrix
    R_x = @(x) [1 0 0; 0 cos(x) -sin(x); 0 sin(x) cos(x)]; % B to v2
    R_y = @(x) [cos(x) 0 sin(x); 0 1 0; -sin(x) 0 cos(x)]; % v2 to v1
    R_z = @(x) [cos(x) -sin(x) 0; sin(x) cos(x) 0; 0 0 1]; % v1 to I
    
    R_B2E = R_z(Psi)*R_y(Theta)*R_x(Phi); % Rotation matrix Body to Earth frame
    
    % Transfer matrix
    T_B2E = (1/cos(Theta))*[cos(Theta) sin(Theta)*sin(Phi)  sin(Theta)*cos(Phi);
        0 cos(Theta)*cos(Phi) -cos(Theta)*sin(Phi);
        0            sin(Phi)             cos(Phi)];
    
    % Skew-symmetric matrix
    S = @(x) [0 -x(3) x(2); x(3) 0 -x(1); -x(2) x(1) 0];
    
    M = [mm*eye(3) zeros(3); zeros(3) I_B*eye(3)];
    C = [zeros(3) -S(mm*Velocity_Lin_B); zeros(3) -S(I_B*Velocity_Ang_B)];
    G = [R_B2E'*[0; 0; mm*grav]; zeros(3,1)];
    U_B = [0; 0; -F; T_1; T_2; T_3];
    
    dy(1:3) = R_B2E*Velocity_Lin_B;
    dy(4:6) = T_B2E*Velocity_Ang_B;
    dy(7:12) = inv(M)*(-C*Velocity_B + G + U_B);
end
%-------------------------------------------------------------------------%

sPhi = sin(Phi); cPhi = cos(Phi);
sTheta = sin(Theta); cTheta = cos(Theta);
sPsi = sin(Psi); cPsi = cos(Psi);

% --- MODEL 2 ------------------------------------------------------------%
if (QQ == 2)    
    dy(1) = y(2);
    dy(2) = (sPsi*sPhi + cPsi*sTheta*cPhi) * (-F/mm);
    
    dy(3) = y(4);
    dy(4) = (-cPsi*sPhi + sPsi*sTheta*cPhi) * (-F/mm);
    
    dy(5) = y(6);
    dy(6) = grav + (cTheta*cPhi)*(-F/mm);
    
    dy(7) = y(8);
    dy(8) = ((Iyy-Izz)/Ixx)*dTheta*dPsi + (T_1/Ixx);
    
    dy(9) = y(10);
    dy(10) = ((Izz-Ixx)/Iyy)*dPhi*dPsi + (T_2/Iyy);
    
    dy(11) = y(12);
    dy(12) = ((Ixx-Iyy)/Izz)*dPhi*dTheta + (T_3/Izz);
end
%-------------------------------------------------------------------------%

% --- MODEL 3 ------------------------------------------------------------%
if (QQ == 3)    
    dy(1) = y(2);
    dy(2) = Theta * (-F/mm);
    
    dy(3) = y(4);
    dy(4) = -Phi * (-F/mm);
    
    dy(5) = y(6);
    dy(6) = grav - F/mm;
    
    dy(7) = y(8);
    dy(8) = T_1/Ixx;
    
    dy(9) = y(10);
    dy(10) = T_2/Iyy;
    
    dy(11) = y(12);
    dy(12) = T_3/Izz;
end
%-------------------------------------------------------------------------%

% --- MODEL 4 ------------------------------------------------------------%
if (QQ == 4)    
    dy(1) = y(2);
    dy(2) = -Theta * grav;
    
    dy(3) = y(4);
    dy(4) = Phi * grav;
    
    dy(5) = y(6);
    dy(6) = grav - F/mm;
    
    dy(7) = y(8);
    dy(8) = T_1/Ixx;
    
    dy(9) = y(10);
    dy(10) = T_2/Iyy;
    
    dy(11) = y(12);
    dy(12) = T_3/Izz;
end
%-------------------------------------------------------------------------%

% --- Other first order differential equations definitions ---------------%
dy(13) = F;
dy(14) = T_1;
dy(15) = T_2;
dy(16) = T_3;

% PID integral part
dy(17) = e_z;
dy(18) = Phi;
dy(19) = Theta;
dy(20) = Psi;

if (YY == 5) % super-twisting tracking control law integral part
    dy(21) = -Uz*sign(s0);
else if (YY == 7)
    dy(21) = -Uz*sign(s0);
    dy(22) = -Uy*sign(s1);
    dy(23) = -Ux*sign(s2);
    dy(24) = -Upsi*sign(s3);
    end
end

% SMOOTHING FILTERS
if (SF == 0) % Z reference w/o smoothing filter
end
if (SF == 1)||(SF == 2)||(SF == 3)
    dy(25) = dz_df;
    dy(26) = dy_df;
    dy(27) = dx_df;
    
    dy(28) = ddz_df;
    dy(29) = ddy_df;
    dy(30) = ddx_df;
    
    dy(31) = d3z_d;
    dy(32) = d3y_d;
    dy(33) = d3x_d;
    
    dy(34) = d4z_d;
    dy(35) = d4y_d;
    dy(36) = d4x_d;
end

% ERROR DERIVATIVE ESTIMATORS
dy(37) = de_z_est; % first order differentiator (velocity estimate)
dy(38) = de_y_est; % first order differentiator (velocity estimate)
dy(39) = de_x_est; % first order differentiator (velocity estimate)

dy(40) = -Ke_st*sign(y(37)-e_z); % part of super-twisting estimator
dy(41) = -Ke_st*sign(y(38)-e_y); % part of super-twisting estimator
dy(42) = -Ke_st*sign(y(39)-e_x); % part of super-twisting estimator

% FOR PLOTS
dy(43) = de_z;
dy(44) = de_y;
dy(45) = de_x;

dy(46) = Omega_orig(1);
dy(47) = Omega_orig(2);
dy(48) = Omega_orig(3);
dy(49) = Omega_orig(4);

dy(50) = Omega(1);
dy(51) = Omega(2);
dy(52) = Omega(3);
dy(53) = Omega(4);

dy(54) = x_ref;
dy(55) = y_ref;
dy(56) = z_ref;

dy(57) = rollrate_sp;
dy(58) = pitchrate_sp;
dy(59) = yawrate_sp;

% Derivatives for controller 8 PIDs
dy(60) = de_dPhi_est;
dy(61) = de_dTheta_est;
dy(62) = de_dPsi_est;

% Integrals for controller 8 PIDs
dy(63) = e_dPhi;
dy(64) = e_dTheta;
dy(65) = e_dPsi;

dy(66) = e_vel_z;
dy(67) = Phi_sp;
dy(68) = Theta_sp;

dy(69) = vel_x_sp;
dy(70) = vel_y_sp;
dy(71) = vel_z_sp;

% --- LINEAR VELOCITY PID CONTROLLERS --- %
% Derivatives
dy(72) = de_vel_x_est;
dy(73) = de_vel_y_est;
dy(74) = de_vel_z_est;
% Integrals
dy(75) = e_vel_x;
dy(76) = e_vel_y;
dy(77) = e_vel_z;

%-------------------------------------------------------------------------%
end % function QuadroHB
