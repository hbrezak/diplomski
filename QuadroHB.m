function dy = QuadroHB(t,y)

global N T QQ YY DD RR grav mm Ixx Iyy Izz I_B d0 Sg Vx0 Ay0 a1 a2 w1 w2
global k_P k_D kk_P kk_D kk_I k_3 k_2 k_1 k_0 x_d y_d z_d Ke Ksf

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
    
    if (t>3*T/4)    % referentna trajektorija
        z_d=0;
    end
    dz_d=0; ddz_d=0; d3z_d=0; d4z_d=0;     
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
    dx_d=0; ddx_d=0; d3x_d=0; d4x_d=0;
    dy_d=0; ddy_d=0; d3y_d=0; d4y_d=0;
    
    z_d = a1*sin(w1*t) + a2*sin(w2*t);
    dz_d = w1*a1*cos(w1*t) + w2*a2*cos(w2*t);
    ddz_d = -w1^2*a1*sin(w1*t) - w2^2*a2*sin(w2*t);
end
%-------------------------------------------------------------------------%

% --- Error variables ----------------------------------------------------% 
e_x = X-x_d; de_x = dX-dx_d; 
e_y = Y-y_d; de_y = dY-dy_d;
e_z = Z-z_d; de_z = dZ-dz_d;
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
% e_z = Z - y(18); % reference smoothing filter 1st order
% e_z = Z - y(19); % reference smoothing filter 2nd order

%de_z_est = -Ke*(y(17) - e_z); % error derivative estimation
de_z_est = -sqrt(Ke)*sqrt(abs(y(17)-e_z))*sign(y(17)-e_z) + y(26);

% U_0 = m*(grav - k_D*dZ - k_P*e_z); % z velocity is measured (dZ known)
U_0 = m*(grav - k_D*de_z_est - k_P*e_z); % velocity is not measured, derivatives are estimated

U_1 = -k_D*dPhi - k_P*Phi;
U_2 = -k_D*dTheta - k_P*Theta;
U_3 = -k_D*dPsi - k_P*Psi;
%-------------------------------------------------------------------------%
end

if (YY == 2)
% --- PID controller -----------------------------------------------------%
% e_z = Z - y(18); % reference smoothing filter 1st order
% e_z = Z - y(19); % reference smoothing filter 2nd order

%de_z_est = -Ke*(y(17) - e_z); % error derivative estimation

vv = -Ke*sqrt(abs(y(17)-Z))*sign(y(17)-Z) + y(26);

de_z_est = -sqrt(Ke)*sqrt(abs(y(17)-e_z))*sign(y(17)-e_z) + y(26);
de_z_est = vv - dz_d;

% U_0 = -kk_D*de_z_est - kk_P*e_z - kk_I*y(20); % PID control
U_0 = m*grav -kk_D*de_z_est - kk_P*e_z - kk_I*y(20); % PID control w/ gravity compensation

U_1 = -kk_D*dPhi - kk_P*Phi - kk_I*y(21);
U_2 = -kk_D*dTheta - kk_P*Theta - kk_I*y(22);
U_3 = -kk_D*dPsi - kk_P*Psi - kk_I*y(23);
%-------------------------------------------------------------------------%
end

if (YY == 3)
% --- Trajectory tracking control law ------------------------------------%
e_z = Z - y(18); % reference smoothing filter 1st order
% e_z = Z - y(19); % reference smoothing filter 2nd order

de_z_est = -Ke*(y(17) - e_z); % error derivative estimation

% U_0 = -kk_D*de_z_est - kk_P*e_z - kk_I*y(20); % PID control
U_0 = m*grav -kk_D*de_z_est - kk_P*e_z - kk_I*y(20); % PID control w/ gravity compensation

dde_x = (grav/m)*Theta - ddx_d;
d3e_x = (grav/m)*dTheta - d3x_d;

dde_y = (-grav/m)*Phi - ddy_d;
d3e_y = (-grav/m)*dPhi - d3y_d;

U_1 = ((-m*Ix)/grav)*(d4y_d - k_3*d3e_y - k_2*dde_y - k_1*de_y - k_0*e_y);
U_2 = ((m*Iy)/grav)*(d4x_d - k_3*d3e_x - k_2*dde_x - k_1*de_x - k_0*e_x);
U_3 = -kk_D*dPsi - kk_P*Psi - kk_I*y(23);
%-------------------------------------------------------------------------%
end

if (YY == 4)
% --- Sliding mode 1st order (sign) --------------------------------------%
% e_z = Z - y(18); % reference smoothing filter 1st order
e_z = Z - y(19); % reference smoothing filter 2nd order

de_z_est = -Ke*(y(17) - e_z); % error derivative estimation

p = 1; eps = 0.01;
U = 20; % 4 values tested: 20, 50, 100, 150
% s = de_z + p*e_z;
s = de_z_est + p*e_z;

% U_0 = -U*sign(s);
% U_0 = -m*U*sign(s);
% U_0 = m*(grav - U*sign(s));
% U_0 = m*(grav - U*s - U*sign(s));

% U_0 = m*(grav - U*s - U*( s / (abs(s) + eps) ) );

s = de_z_est + (k_P/k_D)*e_z;
U_0 = m*(grav - k_D*s - k_D*sign(s));
% U_0 = m*(grav - k_D*s - k_D*( s / (abs(s) + eps) ) );

U_1 = -kk_D*dPhi - kk_P*Phi - kk_I*y(21);
U_2 = -kk_D*dTheta - kk_P*Theta - kk_I*y(22);
U_3 = -kk_D*dPsi - kk_P*Psi - kk_I*y(23);
%-------------------------------------------------------------------------%
end

if (YY == 5)
% --- Super-twisting --------------------------------------%
e_z = Z - y(18); % reference smoothing filter 1st order
% e_z = Z - y(19); % reference smoothing filter 2nd order

de_z_est = -Ke*(y(17) - e_z); % error derivative estimation

p = 1; U = 20; % 20, 50, 80, 100
% s = de_z + p*e_z;
% s = de_z_est + p*e_z;
s = de_z_est + (k_P/k_D)*e_z;

Pn = -sqrt(U) * sqrt(abs(s)) * sign(s);
dIn = -1.1 * U * sign(s);
In = y(25);

% U_0 = Pn + In; % add pure super-twisting
% U_0 = m*(Pn + In);
% U_0 = m*(grav + Pn + In);
% U_0 = m*(grav - U*s + Pn + In);

%s = de_z_est + (k_P/k_D)*e_z;
U_0 = m*(grav - k_D*s + Pn + In);

U_1 = -kk_D*dPhi - kk_P*Phi - kk_I*y(21);
U_2 = -kk_D*dTheta - kk_P*Theta - kk_I*y(22);
U_3 = -kk_D*dPsi - kk_P*Psi - kk_I*y(23);
%-------------------------------------------------------------------------%
end

% #TODO: saturacija motora ---------------
% probably good with l = 0.1; d = 0.0000001; b=0.0000008;
% E_B = [b b b b; 0 -l*b 0 l*b; -l*b 0 l*b 0; -d d -d d];
% Omega = (inv(E_B)*[U_0 U_1 U_2 U_3]');
% kg = 1885^2; % max motor ang. velocity (rad/s), original 2400kV*11.1V reduced by more than 20% for prop
% Omega = kg.*tanh(Omega./kg);
% FF = E_B * Omega;
% ----------------------------------------

% --- Disturbances (wind gust) -------------------------------------------%
if (DD == 0)
    d_0=0; d_1=0; d_2=0; d_3=0;
end

if (DD == 1)
    d_0 = 1*d0*exp(-Sg*(t-T/2)^2);
    d_1 = 1*d0*exp(-Sg*(t-T/2)^2);
    d_2 = 0*d0*exp(-Sg*(t-T/2)^2);
    d_3 = 0*d0*exp(-Sg*(t-T/2)^2);
end

if (DD == 2)
    d_0 = 1*(d0*exp(-Sg*(t+5-1*T/4).^2) + d0*exp(-Sg*(t+5-2*T/4).^2) + d0*exp(-Sg*(t+5-3*T/4).^2) + d0*exp(-Sg*(t+5-4*T/4).^2));
    d_1 = 1*(d0*exp(-Sg*(t+5-1*T/4).^2) + d0*exp(-Sg*(t+5-2*T/4).^2) + d0*exp(-Sg*(t+5-3*T/4).^2) + d0*exp(-Sg*(t+5-4*T/4).^2));
    d_2 = 1*(d0*exp(-Sg*(t+5-1*T/4).^2) + d0*exp(-Sg*(t+5-2*T/4).^2) + d0*exp(-Sg*(t+5-3*T/4).^2) + d0*exp(-Sg*(t+5-4*T/4).^2));
    d_3 = 0*(d0*exp(-Sg*(t+5-1*T/4).^2) + d0*exp(-Sg*(t+5-2*T/4).^2) + d0*exp(-Sg*(t+5-3*T/4).^2) + d0*exp(-Sg*(t+5-4*T/4).^2));
end

if (DD == 3)
    d_0 = 1*(d0*exp(-Sg*(t+5-1*T/4).^2) - d0*exp(-Sg*(t+5-2*T/4).^2) + d0*exp(-Sg*(t+5-3*T/4).^2) - d0*exp(-Sg*(t+5-4*T/4).^2));
    d_1 = 1*(d0*exp(-Sg*(t+5-1*T/4).^2) - d0*exp(-Sg*(t+5-2*T/4).^2) + d0*exp(-Sg*(t+5-3*T/4).^2) - d0*exp(-Sg*(t+5-4*T/4).^2));
    d_2 = 0*(d0*exp(-Sg*(t+5-1*T/4).^2) - d0*exp(-Sg*(t+5-2*T/4).^2) + d0*exp(-Sg*(t+5-3*T/4).^2) - d0*exp(-Sg*(t+5-4*T/4).^2));
    d_3 = 0*(d0*exp(-Sg*(t+5-1*T/4).^2) - d0*exp(-Sg*(t+5-2*T/4).^2) + d0*exp(-Sg*(t+5-3*T/4).^2) - d0*exp(-Sg*(t+5-4*T/4).^2));
end
%-------------------------------------------------------------------------%   

% --- Final control signals:
F = U_0 + d_0;
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
G = [R_B2E'*[0; 0; -mm*grav]; zeros(3,1)];
U_B = [0; 0; F; T_1; T_2; T_3];

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
dy(2) = (sPsi*sPhi + cPsi*sTheta*cPhi) * (F/mm);

dy(3) = y(4);
dy(4) = (-cPsi*sPhi + sPsi*sTheta*cPhi) * (F/mm);

dy(5) = y(6);
dy(6) = -grav + (cTheta*cPhi)*(F/mm);

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
dy(2) = Theta * (F/mm);

dy(3) = y(4);
dy(4) = -Phi * (F/mm);

dy(5) = y(6);
dy(6) = -grav + F/mm;

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
dy(2) = Theta * grav;

dy(3) = y(4);
dy(4) = -Phi * grav;

dy(5) = y(6);
dy(6) = -grav + F/mm;

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
dy(27) = de_z;

dy(17) = vv; % first order differentiator (velocity estimate)
dy(18) = -Ksf*(y(18) - z_d); % 1st order smoothing filter
dy(19) = -Ksf*(y(19) - y(18)); % 2nd order smoothing filter

% PID integral part
dy(20) = e_z;
dy(21) = Phi;
dy(22) = Theta;
dy(23) = Psi;

if (YY == 4) || (YY == 5)
    dy(24) = s;    
end

if (YY == 5)
    dy(25) = dIn;
end
dy(26) = -Ke*sign(y(17)-Z);




end % function QuadroHB




