%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Wind Engineering a.a.2023/2024
% Cross wind on vehicle - M Asperti
% vehicle data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% The lambda factor is to be modified in the range [0,1] and is
% representative of the payload carried by the truck as a fraction of its
% maximum admissible payload
lambda  = 1;            % [-]       relative payload of the vehicle
% The tau_roll factor is to be modified in the range [0.25,0.75] and is
% representative of the ratio of the roll stiffness of the front axle with
% respect to the total roll stiffness of the vehicle (this parameter
% influences the distribution of load transfer among the axles but not the
% total load transfer)
tau_roll = 0.644;       % [-]       front-to-total axle roll stiffness
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% vehicle data: geometry, mass, stiffness and damping
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
g       = 9.80665;      % [m/s2]    gravity
p       = 4.42;         % [m]       Wheel-base
c       = 1.74;         % [m]       track width
Rr      = 0.4;          % [m]       wheel rolling radius
mb      = 4842;         % [kg]      vehicle body mass
Jxb     = 1016;         % [kgm2]    vehicle body moment of inertia x axis
Jzb     = 4795;         % [kgm2]    vehicle body moment of inertia z axis
ab      = 0.534;        % [m]       distance of the body cog from front axle
zb      = 1.058;        % [m]       height of the body cog from ground
al      = 3.35;         % [m]       distance of the lumped mass cog from front axle
ll      = 6.16;         % [m]       length of cargo bed
wl      = 2.5;          % [m]       width of cargo bed
hl      = 2.62;         % [m]       height of cargo bed
zl_min  = 0.88;         % [m]       minimum height of the lumped mass cog
zl_max  = zl_min+hl/2;  % [m]       maximum height of the lumped mass cog
zl      = zl_min+lambda*(zl_max-zl_min);    % [m]       lumped mass cog height
ml_max  = 8600-mb;                    % [kg]      maximum lumped mass
ml      = ml_max*lambda;                    % [kg]      lumped mass simulating the load 
Jxl     = ml/12*(wl^2+(lambda*hl)^2);       % [kgm2]    lumped mass moment of inertia x axis
Jzl     = ml/12*(ll^2+wl^2);                % [kgm2]    lumped mass moment of inertia z axis
m       = mb+ml;              % [kg]      total sprung mass
a       = (al*ml+ab*mb)/m;    % [m]       distance of sprung mass cog from front axle
b       = p-a;                % [m]       distance of sprung mass cog from rear axle
zG      = (zb*mb+zl*ml)/(mb+ml);    % [m]    height of sprung mass cog from ground
Jz      = Jzb+mb*(a-ab)^2+Jzl+ml*(a-al)^2;   % total moment of inerzia z
Jx      = Jxb + mb*(zG-zb)^2 + Jxl + ml*(zG-zl)^2;   % total moment of inerzia x
Fza     = m*b/p*g;          % [N]       static load on front wheels
Fzp     = m*a/p*g;          % [N]       static load on rear wheels
zR      = Rr*1.25;           % [m]       roll centre height from ground
c_roA   = 4*0.5*10.36*1e3;   % [Nms/rad] roll damping front axle
c_roP   = 2*0.5*6.28*1e3;  % [Nms/rad] roll damping rear axle
k_roTOT = 1.5*(252+139)*1e3;    % [Nm/rad]  total vehicle roll stiffness
k_roA   = k_roTOT*tau_roll;      % [Nm/rad]  roll stiffness front axle
k_roP   = k_roTOT*(1-tau_roll);      % [Nm/rad]  roll stiffness rear axle
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Pacejka coefficients and driver gains
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
C = 1.5;
E = -0.5;
% By selecting the road surface condition, the tire-road contact forces
% parametes get updated accordingly
road_select = menu('Select road conditions','Snow','Wet','Dry');
if road_select==1
    q = 0.3;
elseif road_select==2
    q = 0.6;
elseif road_select==3
    q = 0.9;
else
    warning('Error in road condition selection')
end
s = 0;-0.01;
N0 = 20000; %[N]
BF = 2.7502; % da CalcoloRigidezzeDeriva.m
BR = 2.5*BF;