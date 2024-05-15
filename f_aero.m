function [Fy,Mx,Mz,alfa,Uy,Vr] = forze_aerodinamiche(Xp,Yp,psi,X,Y,time)

% inputs
% Xp    vehicle velocity along X absolute axis
% Yp    vehicle velocity along Y absolute axis
% psi   vehicle heading with respect to X absolute axis
% X     vehicle X coordinate in absolute X axis
% Y     vehicle Y coordinate in absolute Y axis
% t     actual simulation time
% outputs
% Fy    aerodynamic force along vehicle y axis
% Mx    aerodynamic moment along vehicle roll axis (x)
% Mz    aerodynamic moment along vehicle yaw axis (Z)
% alfa  angle of attach
% Uy    wind speed seen by the vehicle
% Vr    relative wind speed magnitude
global Wind_Coefficients vento Al hr den
% Wind absolute speed as function of time and position
a1 = floor(time / (vento.t(2)-vento.t(1))) + 1;
a2= floor(X / (vento.x(2)-vento.x(1))) + 1;
Uy = vento.v(a1,a2);

% this is made to avoid numerical instability at the beginning of the
% simulation
if time < 2
    Uy = Uy*time/2;
end
% relative Wind speed in absolute reference frame
Vrx =  - Xp;
Vry = Uy-Yp;
Vr  = sqrt(Vrx.^2 + Vry.^2); % modulus
% relative Wind speed in local reference frame
vrx = Vrx*cos(psi) + Vry*sin(psi);
vry = -Vrx*sin(psi)+Vry*cos(psi);
% Wind yaw angle
alfa  = 180 / pi * atan(-vry/vrx);
% coefficients interpolation from table. C = [Cx Cy Mz Mx];

Cx = interp1(Wind_Coefficients(1,:), Wind_Coefficients(2,:), alfa);
Cy = interp1(Wind_Coefficients(1,:),Wind_Coefficients(3,:), alfa);
Cmz = interp1(Wind_Coefficients(1,:),Wind_Coefficients(4,:), alfa);
Cmx = interp1(Wind_Coefficients(1,:), Wind_Coefficients(5,:), alfa);

% Aerodynamic forces
Fy  = 1/2*den*Al*Cy*Vr^2;
Mx  = 1/2*den*Al*hr*Vr^2*Cmx;
Mz  = 1/2*den*Al*hr*Vr^2*Cmz;