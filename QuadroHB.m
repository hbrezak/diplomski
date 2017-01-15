function dy = QuadroHB(t,y)

global N T QQ YY DD RR SF grav mm Ixx Iyy Izz I_B d0 Sg Vx0 Ay0 a1 a2 w1 w2 stepAmp
global k_P k_D kk_P kk_D kk_I k_3 k_2 k_1 k_0 x_d y_d z_d Ke_lin Ke_st Ksf rho u kg 
global E_B inv_E_B AngVel_limit

dy = zeros(N, 1);

if (QQ == 1)
% --- STATE VARIABLES - MODEL 1 ------------------------------------------%
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
%-------------------------------------------------------------------------%
end

if (QQ == 2)||(QQ == 3)||(QQ == 4)
% --- STATE VARIABLES - MODEL 2 - MODEL 3 - MODEL 4 ----------------------%
X=y(1); dX=y(2);
Y=y(3); dY=y(4);
Z=y(5); dZ=y(6);

Phi=y(7); dPhi=y(8);
Theta=y(9); dTheta=y(10);
Psi=y(11); dPsi=y(12);
%-------------------------------------------------------------------------%
end

% --- Reference trajectory parameters ------------------------------------%
if (RR == 1)
    dx_d=0; ddx_d=0; d3x_d=0; d4x_d=0;
    dy_d=0; ddy_d=0; d3y_d=0; d4y_d=0;
    
    % what ever T is, set step to start at 1 sec. and lower it to 0 at last quarter  
    if(t<1)
        z_d = 0;    
    else if (t>3*T/4)    % referentna trajektorija
        z_d = 0;
        else
            z_d = 1;
        end
    end
    z_d = z_d * stepAmp;
    dz_d=0; ddz_d=0; d3z_d=0; d4z_d=0;     
end
if (RR == 2)
    x_d = -Ay0*0 + Ay0*cos(Vx0*t);
    y_d = Ay0*sin(Vx0*t);
    z_d = Vx0*t;    
    
%     dx_d = -Ay0*Vx0*sin(Vx0*t); ddx_d = -Ay0*Vx0^2*cos(Vx0*t); d3x_d = Ay0*Vx0^3*sin(Vx0*t); d4x_d = Ay0*Vx0^4*cos(Vx0*t);
%     dy_d = Ay0*Vx0*cos(Vx0*t); ddy_d = -Ay0*Vx0^2*sin(Vx0*t); d3y_d = -Ay0*Vx0^3*cos(Vx0*t); d4y_d = Ay0*Vx0^4*sin(Vx0*t);
%     dz_d = Vx0; ddz_d = 0; d3z_d = 0; d4z_d = 0;     

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
if (SF == 0) % Z reference w/o smoothing filter
    z_df = z_d;
    dz_df = dz_d;
    
    y_df = y_d;
    dy_df = dy_d;
    
    x_df = x_d;
    dx_df = dx_d;
end
if (SF == 1) % Z reference w/ smoothing filter 1st order
    z_df = y(18);
    dz_df = -Ksf*(y(18) - z_d);
    
    y_df = y(33);
    dy_df = -Ksf*(y(33) - y_d);
    
    x_df = y(34);
    dx_df = -Ksf*(y(34) - x_d);
end
if (SF == 2) % Z reference w/ smoothing filter 2nd order
    z_df = y(19);
    dz_df = -Ksf*(y(19) - y(18));
    
    y_df = y(35);
    dy_df = -Ksf*(y(35) - y(33));
    
    x_df = y(36);
    dx_df = -Ksf*(y(36) - y(34));
    
    ddz_d = -Ksf*(y(39) - dz_df);
    ddy_d = -Ksf*(y(40) - dy_df);
    ddx_d = -Ksf*(y(41) - dx_df);
    
    d3z_d = -Ksf*(y(42) - ddz_d);
    d3y_d = -Ksf*(y(43) - ddy_d);
    d3x_d = -Ksf*(y(44) - ddx_d);
    
    d4z_d = -Ksf*(y(45) - d3z_d);
    d4y_d = -Ksf*(y(46) - d3y_d);
    d4x_d = -Ksf*(y(47) - d3x_d);
end
if (SF == 3) % Z reference w/ nonlinear saturated smoothing filter
    z_df = y(27);
    dz_df = -rho*tanh(u*(y(27) - z_d));
    
    y_df = y(37);
    dy_df = -rho*tanh(u*(y(37) - y_d));
    
    x_df = y(38);
    dx_df = -rho*tanh(u*(y(38) - x_d));
end
%-------------------------------------------------------------------------%


% --- Error variables ----------------------------------------------------% 
e_x = X-x_df; de_x = dX-dx_df; 
e_y = Y-y_df; de_y = dY-dy_df;
e_z = Z-z_df; de_z = dZ-dz_df;
%-------------------------------------------------------------------------%

% --- Quadrotor parameters(nominal) --------------------------------------%
% Parametri: m, Ix, Iy, Iz su pretpostavljene vrijednosti realnih
% parametara mm, Ixx, Iyy, Izz koje koristimo u kontroleru (robusnost)
m = 1.0*mm; 
Ix = 1.0*Ixx; 
Iy = 1.0*Iyy; 
Iz = 1.0*Izz; 
%-------------------------------------------------------------------------%

if (YY == 1)
% --- PD controller ------------------------------------------------------%

% de_z_est = de_z;
de_z_est = -Ke_lin*(y(17) - e_z); % error derivative estimation
% de_z_est = -Ke_st*sqrt(abs(y(17)-e_z))*sign(y(17)-e_z) + y(26); %super-twisting derivative estimator

% U_0 = m*(grav + k_D*dZ + k_P*e_z); % z velocity is measured (dZ known)
% U_0 = m*(grav + k_D*de_z_est + k_P*e_z); % velocity is not measured, derivatives are estimated
U_0 = max((m*(grav + k_D*de_z_est + k_P*e_z)), 0); % limit to positive numbers only

U_1 = -k_D*dPhi - k_P*Phi;
U_2 = -k_D*dTheta - k_P*Theta;
U_3 = -k_D*dPsi - k_P*Psi;
%-------------------------------------------------------------------------%
end

if (YY == 2)
% --- PID controller -----------------------------------------------------%

de_z_est = -Ke_lin*(y(17) - e_z); % 1st order filter error derivative estimation
% de_z_est = -Ke_st*sqrt(abs(y(17)-e_z))*sign(y(17)-e_z) + y(26); %super-twisting derivative estimator

% U_0 = kk_D*de_z_est + kk_P*e_z + kk_I*y(20); % PID control
% U_0 = m*grav +kk_D*de_z_est + kk_P*e_z + kk_I*y(20); % PID control w/ gravity compensation
U_0 = max((m*grav +kk_D*de_z_est + kk_P*e_z + kk_I*y(20)), 0); % limit to positive numbers only

U_1 = -kk_D*dPhi - kk_P*Phi - kk_I*y(21);
U_2 = -kk_D*dTheta - kk_P*Theta - kk_I*y(22);
U_3 = -kk_D*dPsi - kk_P*Psi - kk_I*y(23);
%-------------------------------------------------------------------------%
end

if (YY == 3)
% --- Trajectory tracking control law ------------------------------------%

de_z_est = -Ke_lin*(y(17) - e_z); % error derivative estimation
% de_z_est = -Ke_st*sqrt(abs(y(17)-e_z))*sign(y(17)-e_z) + y(26); %super-twisting derivative estimator

% U_0 = kk_D*de_z_est + kk_P*e_z + kk_I*y(20); % PID control
% U_0 = m*grav +kk_D*de_z_est + kk_P*e_z + kk_I*y(20); % PID control w/ gravity compensation
U_0 = max(-m*(-grav + ddz_d -kk_D*de_z_est - kk_P*e_z - kk_I*y(20)), 0); % limit to positive numbers only

dde_x = (-grav/m)*Theta - ddx_d;
d3e_x = (-grav/m)*dTheta - d3x_d;

dde_y = (grav/m)*Phi - ddy_d;
d3e_y = (grav/m)*dPhi - d3y_d;

U_1 = ((m*Ix)/grav)*(d4y_d - k_3*d3e_y - k_2*dde_y - k_1*de_y - k_0*e_y);
U_2 = ((-m*Iy)/grav)*(d4x_d - k_3*d3e_x - k_2*dde_x - k_1*de_x - k_0*e_x);
U_3 = Iz*(-kk_D*dPsi - kk_P*Psi - kk_I*y(23));
%-------------------------------------------------------------------------%
end

if (YY == 4)
% --- Sliding mode 1st order (sign) --------------------------------------%

de_z_est = -Ke_lin*(y(17) - e_z); % error derivative estimation
% de_z_est = -Ke_st*sqrt(abs(y(17)-e_z))*sign(y(17)-e_z) + y(26); %super-twisting derivative estimator

p = 1; eps = 0.01;
U = 20; % 4 values tested: 20, 50, 100, 150
% s = de_z + p*e_z;
s = de_z_est + p*e_z;

% U_0 = U*sign(s);
% U_0 = m*U*sign(s);
% U_0 = m*(grav + U*sign(s));
% U_0 = m*(grav + U*s - U*sign(s));

% U_0 = m*(grav + U*s + U*( s / (abs(s) + eps) ) );

U_0 = max(m*(grav + U*s + U*sign(s)), 0); % limit to positive numbers only

% s = de_z_est + (k_P/k_D)*e_z;
% U_0 = m*(grav + k_D*s + k_D*sign(s));
% U_0 = m*(grav + k_D*s + k_D*( s / (abs(s) + eps) ) );
% U_0 = max((m*(grav + k_D*s + k_D*( s / (abs(s) + eps) ) )), 0); % limit to positive numbers only

U_1 = -kk_D*dPhi - kk_P*Phi - kk_I*y(21);
U_2 = -kk_D*dTheta - kk_P*Theta - kk_I*y(22);
U_3 = -kk_D*dPsi - kk_P*Psi - kk_I*y(23);
%-------------------------------------------------------------------------%
end

if (YY == 5)
% --- Super-twisting --------------------------------------%

% de_z_est = de_z; % use measured velocity
% de_z_est = -Ke_lin*(y(17) - e_z); % 1st order filter error derivative estimation
de_z_est = -Ke_st*sqrt(abs(y(17)-e_z))*sign(y(17)-e_z) + y(26); %super-twisting derivative estimator

p = 3; U = 20; % 20, 50, 80, 100

s = de_z_est + p*e_z;
% s = de_z_est + (k_P/k_D)*e_z;

ST = -U*sqrt(abs(s))*sign(s) + y(25);
% ST = -sqrt(U)*sqrt(abs(s))*sign(s) + y(25); % sa U=100 i vise daje ok rez ali Fz je kratko negativan


% U_0 = ST; % add pure super-twisting
% U_0 = m*ST;
% U_0 = m*(grav + ST);
% U_0 = m*(grav - U*s + ST);
% U_0 = m*(grav - k_D*s) + ST;

%k_m = 4;
%U_0 = m*grav + k_m*tanh( -(m/k_m)*U*s + (1/k_m)*ST);

U_0 = max((m*grav + m*U*s - ST), 0); % limit to positive numbers only

U_1 = -kk_D*dPhi - kk_P*Phi - kk_I*y(21);
U_2 = -kk_D*dTheta - kk_P*Theta - kk_I*y(22);
U_3 = -kk_D*dPsi - kk_P*Psi - kk_I*y(23);
%-------------------------------------------------------------------------%
end

if (YY == 6)
% --- 1-SM Trajectory tracking control law ------------------------------------%

de_z_est = -Ke_lin*(y(17) - e_z); % error derivative estimation
% de_z_est = -Ke_st*sqrt(abs(y(17)-e_z))*sign(y(17)-e_z) + y(26); %super-twisting derivative estimator

dde_x = -Theta*grav - ddx_d;
d3e_x = -dTheta*grav - d3x_d;

dde_y = Phi*grav - ddy_d;
d3e_y = dPhi*grav - d3y_d;

%U = 20;

% Sliding surfaces
% p = 1; lambda = 1;
% s0 = de_z + p*e_z;
% s1 = d3e_y + 3*p*dde_y + 3*(p^2)*de_y + (p^3)*e_y;
% s2 = d3e_x + 3*p*dde_x + 3*(p^2)*de_x + (p^3)*e_x;
% s3 = dPsi + p*Psi;
% 
% kx3 = 3*p + 1; kx2 = 3*(p^2)+1*3*p; kx1 = (p^3)+1*3*(p^2); kx0 = 1*(p^3);

lambda = 3; p = 3; K = 20;
alpha_z0 = p;
alpha_x0 = p^3; alpha_x1 = 3*p^2; alpha_x2 = 3*p;
alpha_y0 = p^3; alpha_y1 = 3*p^2; alpha_y2 = 3*p;
alpha_psi0 = p;

Uz = 0; Ux = 4; Uy = 4; Upsi = 0;
% Z
s0 = de_z + alpha_z0*e_z;
k_z1 = alpha_z0 + lambda;
k_z0 = alpha_z0*lambda;

%X
s2 = d3e_x + alpha_x2*dde_x + alpha_x1*de_x + alpha_x0*e_x;
k_x3 = alpha_x2 + lambda;
k_x2 = alpha_x1 + lambda*alpha_x2;
k_x1 = alpha_x0 + lambda*alpha_x1;
k_x0 = lambda * alpha_x0;

%Y
s1 = d3e_y + alpha_y2*dde_y + alpha_y1*de_y + alpha_y0*e_y;
k_y3 = alpha_y2 + lambda;
k_y2 = alpha_y1 + lambda*alpha_y2;
k_y1 = alpha_y0 + lambda*alpha_y1;
k_y0 = lambda * alpha_y0;

% PSI
s3 = dPsi + alpha_psi0*Psi;
k_psi1 = alpha_psi0 + lambda;
k_psi0 = alpha_psi0 * lambda;



% 1-SM controllers
% SM_0 = -Uz*sign(s0);
% SM_1 = -Uy*sign(s1);
% SM_2 = -Ux*sign(s2);
% SM_PSI = -Upsi*sign(s3);

SM_0 = -Uz*tanh(K*s0);
SM_1 = -Uy*tanh(K*s1);
SM_2 = -Ux*tanh(K*s2);
SM_PSI = -Upsi*tanh(K*s3);

% Control laws
U_0 = max((-m*(ddz_d - grav -k_z1*de_z - k_z0*e_z) - SM_0), 0);
U_1 = (Ix/grav)*(d4y_d - k_y3*d3e_y - k_y2*dde_y - k_y1*de_y - k_y0*e_y) + SM_1;
U_2 = (-Iy/grav)*(d4x_d - k_x3*d3e_x - k_x2*dde_x - k_x1*de_x - k_x0*e_x) -SM_2;
U_3 = Iz*(-k_psi1*dPsi - k_psi0*Psi) + SM_PSI;
%-------------------------------------------------------------------------%
end

if (YY == 7)
% --- Super-twisting Trajectory tracking control law ------------------------------------%

%de_z_est = -Ke_lin*(y(17) - e_z); % error derivative estimation
de_y_est = -Ke_lin*(y(59) - e_y); % error derivative estimation
de_x_est = -Ke_lin*(y(60) - e_x); % error derivative estimation

L = 1; lam1 = 1.5*sqrt(L); lam0 = 1.1*L; 
de_z_est = -lam1*sqrt(abs(y(17)-e_z))*sign(y(17)-e_z) + y(26); %super-twisting derivative estimator
% de_y_est = -lam1*sqrt(abs(y(59)-e_y))*sign(y(59)-e_y) + y(61); %super-twisting derivative estimator
% de_x_est = -lam1*sqrt(abs(y(60)-e_x))*sign(y(60)-e_x) + y(62); %super-twisting derivative estimator
% 
% de_z_est = de_z;
% de_y_est = de_y;
% de_x_est = de_x;

dde_x = -Theta*grav - ddx_d;
d3e_x = -dTheta*grav - d3x_d;

dde_y = Phi*grav - ddy_d;
d3e_y = dPhi*grav - d3y_d;

% Sliding surfaces
lambda = 3; p = 3;
lambdaz = 1; pz = 1;
alpha_z0 = pz;
alpha_x0 = p^3; alpha_x1 = 3*p^2; alpha_x2 = 3*p;
alpha_y0 = p^3; alpha_y1 = 3*p^2; alpha_y2 = 3*p;
alpha_psi0 = p;

Uz = 1; Ux = 4; Uy = 4; Upsi = 4;
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
k_psi1 = alpha_psi0 + lambda;
k_psi0 = alpha_psi0 * lambda;

c0 = 1.5*sqrt(Uz); b0 = 1.1*Uz;
c1 = 1.5*sqrt(Uy); b1 = 1.1*Uy;
c2 = 1.5*sqrt(Ux); b2 = 1.1*Ux;
c3 = 1.5*sqrt(Upsi); b3 = 1.1*Upsi;


% ST_0 = -Uz*sqrt(abs(s0))*sign(s0) + y(48);
% ST_1 = -Uy*sqrt(abs(s1))*sign(s1) + y(49);
% ST_2 = -Ux*sqrt(abs(s2))*sign(s2) + y(50);
% ST_PSI = -Upsi*sqrt(abs(s3))*sign(s3) + y(51);

ST_0 = -c0*sqrt(abs(s0))*sign(s0) + y(48);
ST_1 = -c1*sqrt(abs(s1))*sign(s1) + y(49);
ST_2 = -c2*sqrt(abs(s2))*sign(s2) + y(50);
ST_PSI = -c3*sqrt(abs(s3))*sign(s3) + y(51);

% Control laws
u0 = max((-m*(ddz_d - grav -k_z1*de_z_est - k_z0*e_z) - ST_0), 0);
u1 = (Ix/grav)*(d4y_d - k_y3*d3e_y - k_y2*dde_y - k_y1*de_y_est - k_y0*e_y);% + ST_1;
u2 = (-Iy/grav)*(d4x_d - k_x3*d3e_x - k_x2*dde_x - k_x1*de_x_est - k_x0*e_x);% - ST_2;
u3 = Iz*(-k_psi1*dPsi - k_psi0*Psi);% + ST_PSI;

dy(55) = -rho*tanh(u*(y(55) - u0));
dy(56) = -rho*tanh(u*(y(56) - u1));
dy(57) = -rho*tanh(u*(y(57) - u2));
dy(58) = -rho*tanh(u*(y(58) - u3));

% U_0 = y(55);
% U_1 = y(56);
% U_2 = y(57);
% U_3 = y(58);

U_0 = u0;
U_1 = u1;
U_2 = u2;
U_3 = u3;
%-------------------------------------------------------------------------%
end



% --- Actuator saturation ------------------------------------------------%
Omega = real(sqrt(inv_E_B*[U_0 U_1 U_2 U_3]'));
% 
% if (max(Omega) > AngVel_limit)
%     scale_factor = AngVel_limit/max(Omega);
%     Omega = Omega .* scale_factor;
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
    d_0 = 1*d0*exp(-Sg*(t-T/2)^2);
    d_1 = 0*d0*exp(-Sg*(t-T/2)^2);
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

% --- Final control signals:
%F = kg*tanh((U_0 + d_0)/kg);

% F = FF(1) + d_0;
% % fprintf('Sila %f  F_z %f  poremecaj %f \n', F, FF(1), d_0);
% T_1 = FF(2) + d_1;
% T_2 = FF(3) + d_2;
% T_3 = FF(4) + d_3;
% 
F = U_0 + d_0;
% fprintf('Sila %f  F_z %f  poremecaj %f \n', F, FF(1), d_0);
T_1 = U_1 + d_1;
T_2 = U_2 + d_2;
T_3 = U_3 + d_3;


if (QQ == 1)
% --- MODEL 1 ------------------------------------------------------------%
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
%-------------------------------------------------------------------------%
end

sPhi = sin(Phi); cPhi = cos(Phi);
sTheta = sin(Theta); cTheta = cos(Theta);
sPsi = sin(Psi); cPsi = cos(Psi);

if (QQ == 2)
% --- MODEL 2 ------------------------------------------------------------%
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

%-------------------------------------------------------------------------%
end

if (QQ == 3)
% --- MODEL 3 ------------------------------------------------------------%
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

%-------------------------------------------------------------------------%
end

if (QQ == 4)
% --- MODEL 4 ------------------------------------------------------------%
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

%-------------------------------------------------------------------------%
end

dy(13) = F;
dy(14) = T_1;
dy(15) = T_2;
dy(16) = T_3;


dy(17) = de_z_est; % first order differentiator (velocity estimate)
dy(59) = de_y_est; % first order differentiator (velocity estimate)
dy(60) = de_x_est; % first order differentiator (velocity estimate)

%dy(18) = -Ksf*(y(18) - z_d); % 1st order z-reference smoothing filter
%dy(19) = -Ksf*(y(19) - y(18)); % 2nd order z-reference smoothing filter


% SMOOTHING FILTERS
if (SF == 0) % Z reference w/o smoothing filter
end
if (SF == 1) % Z reference w/ smoothing filter 1st order
    dy(18) = dz_df;
    dy(33) = dy_df;
    dy(34) = dx_df;
end
if (SF == 2) % Z reference w/ smoothing filter 2nd order
    dy(18) = -Ksf*(y(18) - z_d);
    dy(19) = dz_df;
    dy(33) = -Ksf*(y(33) - y_d);
    dy(35) = dy_df;
    dy(34) = -Ksf*(y(34) - x_d);
    dy(36) = dx_df;
end
if (SF == 3) % Z reference w/ nonlinear saturated smoothing filter  
    dy(27) = dz_df;
    dy(37) = dy_df;
    dy(38) = dx_df;
end







% PID integral part
dy(20) = e_z;
dy(21) = Phi;
dy(22) = Theta;
dy(23) = Psi;

if (YY == 4)||(YY == 5)
    dy(24) = s;    % for plot
    dy(25) = -U*sign(s); % part of super-twisting algorithm
    % dy(25) = -1.1*U*sign(s); % part of super-twisting algorithm
end
% de_z_est = -lam1*sqrt(abs(y(17)-e_z))*sign(y(17)-e_z) + y(26); %super-twisting derivative estimator
% de_y_est = -lam1*sqrt(abs(y(59)-e_y))*sign(y(59)-e_y) + y(61); %super-twisting derivative estimator
% de_x_est = -lam1*sqrt(abs(y(60)-e_x))*sign(y(60)-e_x) + y(62); %super-twisting derivative estimator

dy(26) = -lam0*sign(y(17)-e_z); % part of super-twisting estimator
dy(61) = -lam0*sign(y(59)-e_y); % part of super-twisting estimator
dy(62) = -lam0*sign(y(60)-e_x); % part of super-twisting estimator

%dy(27) = -rho*tanh(u*(y(27) - z_d)); %nonlinear saturated z-reference smoothing filter

dy(28) = de_z;
dy(63) = de_y;
dy(64) = de_x;

dy(29) = Omega(1);
dy(30) = Omega(2);
dy(31) = Omega(3);
dy(32) = Omega(4);

dy(39) = ddz_d;
dy(40) = ddy_d;
dy(41) = ddx_d;
    
dy(42) = d3z_d;
dy(43) = d3y_d;
dy(44) = d3x_d;
    
dy(45) = d4z_d;
dy(46) = d4y_d;
dy(47) = d4x_d;

if (YY == 7)
%     dy(48) = -Uz*sign(s0);
%     dy(49) = -Uy*sign(s1);
%     dy(50) = -Ux*sign(s2);
%     dy(51) = -Upsi*sign(s3);
    dy(48) = -b0*sign(s0);
    dy(49) = -b1*sign(s1);
    dy(50) = -b2*sign(s2);
    dy(51) = -b3*sign(s3);
end

dy(52) = dx_df;
dy(53) = dy_df;
dy(54) = dz_df;


end % function QuadroHB
