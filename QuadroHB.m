function dy = QuadroHB(t,y)

global N QQ grav mm Ixx Iyy Izz I_B
global k_P k_D

dy = zeros(N, 1);

if (QQ == 1)
% --- STATE VARIABLES - MODEL 1 ------------------------------------------%
X=y(1); %Xd=y(7); #TODO: Fix MODEL 1 
Y=y(2); %Yd=y(8);
Z=y(3); %Zd=y(9);

Phi=y(4); %Phid=y(10);
Theta=y(5); %Thetad=y(11);
Psi=y(6); %Psid=y(12);

Position = [X; Y; Z;];
Angle = [Phi; Theta; Psi];
Velocity_Lin_B = y(7:9);
Velocity_Ang_B = y(10:12);
Velocity = [Velocity_Lin_B; Velocity_Ang_B];
%-------------------------------------------------------------------------%
end

if (QQ == 2)||(QQ == 3)||(QQ == 4)
% --- STATE VARIABLES - MODEL 2 - MODEL 3 - MODEL 4 ----------------------%
X=y(1); Xd=y(2);
Y=y(3); Yd=y(4);
Z=y(5); Zd=y(6);

Phi=y(7); Phid=y(8);
Theta=y(9); Thetad=y(10);
Psi=y(11); Psid=y(12);
%-------------------------------------------------------------------------%
end

% --- Error variables ----------------------------------------------------% 
e_x = X-x_d; d1e_x = Xd-d1x_d; 
e_y = Y-y_d; d1e_y = Yd-d1y_d;
e_z = Z-z_d; d1e_z = Zd-d1z_d;
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
U_0 = m*(grav - k_D*Zd - k_P*e_z); % z velocity is measured (Zd known)

U_1 = -k_D*Phid - k_P*Phi;
U_2 = -k_D*Thetad - k_P*Theta;
U_3 = -k_D*Psid - k_P*Psi;
%-------------------------------------------------------------------------%
end

if (QQ == 1)
% --- MODEL 1 ------------------------------------------------------------%
% Rotation matrix
R_x = @(x) [1 0 0; 0 cos(x) -sin(x); 0 sin(x) cos(x)];
R_y = @(x) [cos(x) 0 sin(x); 0 1 0; -sin(x) 0 cos(x)];
R_z = @(x) [cos(x) -sin(x) 0; sin(x) cos(x) 0; 0 0 1];

R_B2E = R_z(Psi) * R_y(Theta) * R_x(Phi);

% Transfer matrix
T_B2E = (1/cos(Theta))*[cos(Theta) sin(Theta)*sin(Phi)  sin(Theta)*cos(Phi);
                                 0 cos(Theta)*cos(Phi) -cos(Theta)*sin(Phi);
                                 0            sin(Phi)             cos(Phi)];
                      
% Skew-symmetric matrix
S = @(x) [0 -x(3) x(2); x(3) 0 -x(1); -x(2) x(1) 0];

M = [mm*eye(3) zeros(3); zeros(3) I_B*exe(3)];
C = [zeros(3) -S(mm*Velocity_Lin_B); zeros(3) -S(I_B*Velocity_Ang_B)];
G = [R_B2E*[0; 0; -m*grav]; zeros(3,1)];
U_B = [0; 0; F; T_1; T_2; T_3];

dy(1:3) = R_B2E*Velocity_Lin_B;
dy(4:6) = R_B2E*Velocity_Ang_B;
dy(7:12) = inv(M)*(-C*Velocity_B + G + U);
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
dy(4) = -(cPsi*sPhi + sPsi*sTheta*cPhi) * (F/mm);

dy(5) = y(6);
dy(6) = -grav + (cTheta*cPhi)*(F/mm);

dy(7) = y(8);
dy(8) = ((Iy-Iz)/Ix)*Thetad*Psid + (T_1/Ixx);

dy(9) = y(10);
dy(10) = ((Iz-Ix)/Iy)*Phid*Psid + (T_2/Iyy);

dy(11) = y(12);
dy(12) = ((Ix-Iy)/Iz)*Phid*Thetad + (T_3/Izz);

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




