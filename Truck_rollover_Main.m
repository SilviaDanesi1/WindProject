%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Wind Engineering a.a.2023/2024
% Cross wind on vehicle
% 4 contact model
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all
clear all
clc
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% global variables
global g dt
global m Jz Jx Fza Fzp
global a b p c zR zG xP zP Rr
global c_roA c_roP k_roA k_roP dNA dNP
global BF BR C E q s N0
global kp kd ki L x_ref y_ref ErrInt
global Af Al den fid hr
global FyW MzW MxW Wind_Coefficients vento
global indice xp_prec iii xpp
global delta d
global vx
indice = 0;
iii = 2;
xpp(1,:) = zeros(1,7);
delta(1,1) = 0;
dNA(1,1) = 0;
dNP(1,1) = 0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% vehicle data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Truck_rollover_vehicle_data_evo

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% driver gains and reference path
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
kp = 5;                             % [rad/m]   driver proportional gain
kd = .05;                             % [         driver derivative gain
ki = 0.1;
L = 3;                              % [m]       driver prediction lenght
x_path = [0 20 25 70 75 500 1000];       % [m]       driver reference path x
y_path = [0 0   1  1  0    0 0]*0;       % [m]       driver reference path y
x_ref = 0:1: x_path(end);
y_ref = interp1(x_path,y_path,x_ref);
y_ref = smooth(y_ref,20);
ErrInt = 0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Wind Parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Wind data
Af = 6.6;               % [m^2]     front surface
Al = 18.9;              % [m^2]     lateral surface
hr = 2.62;              % [m]       reference height
den = 1.204;             % [kg/m^3]  air density
xP = p/2-b;             % [m]       x coordinate F aero application point
zP = 0;                 % [m]       z coordinate F aero application point
% choose a wind history
hd = helpdlg('choose a wind history file');
waitfor(hd);
[nomefile,cartella]=uigetfile('*.dat');
vento.v = importdata([cartella,nomefile]);
vento.t = vento.v(:,1);
vento.v = vento.v(:,2:end);
vento.x = (0:5:5*(size(vento.v,2)-1))';
load Wind_Coefficients  %           aerodynamic coefficients

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Simulation data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
prompt = {'Vehicle speed [km/h]','Final time [s]','integration step time [s]'};
titolo = 'Simulation data';
def = {'50','10','0.05'};
dati = inputdlg(prompt,titolo,1,def);
V = str2double(char(dati(1)));
vx = V/3.6;
tfin = str2double(char(dati(2)));
dt = str2double(char(dati(3)));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Numeric integration
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% state vector
%       1       2       3       4       5       6       7
% x  = [vy      psip    rhop    X       Y       psi     rho];
% xp = [vyp     psipp   rhopp   Xp      Yp      psip    rhop]
xp_prec = zeros(7,1);
x0 = zeros(7,1); % initial conditions
t0 = 0;
[t,x] = ode45c('Truck_rollover_Equations',t0,tfin,dt,x0);

% accelerations 
vx      = vx*ones(size(t));
vy      = x(:,1);
psip    = x(:,2);
rhop    = x(:,3);
X       = x(:,4);
Y       = x(:,5);
psi     = x(:,6);
rho     = x(:,7);
vyp     = xpp(:,1);
psipp   = xpp(:,2);
rhopp   = xpp(:,3);
ay      = vyp + vx.*psip;
Xp      = xpp(:,4);
Yp      = xpp(:,5);
beta    = atan2(vy,vx); 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% wind forces and moments
for ii=1:length(t)
    [FyW(ii,1),MxW(ii,1),MzW(ii,1),alfa(ii,1),Uy(ii,1),Vr(ii,1)] = f_aero(Xp(ii),Yp(ii),psi(ii),X(ii),Y(ii),t(ii));
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% load on wheels
% load transfer
MA  = k_roA*rho + c_roA*rhop;    % roll moment due to roll stiffness and damping
MP  = k_roP*rho + c_roP*rhop;
dNa = MA / c; % vertical load transfer front axle
dNp = MP / c; % vertical load transfer rear axle
% load on tyres
NFL = Fza/2 - dNa; % normal load front left
NFR = Fza/2 + dNa; % normal load front right
NRL = Fzp/2 - dNp; % normal load rear left
NRR = Fzp/2 + dNp; % normal load rear right


alphaF = delta -atan((vy + psip*a)./vx);
alphaR = -atan((vy-psip*b)./vx);
% pacejka D coefficient
DFL = (q+s.*(NFL-N0)./N0).*NFL; % coefficient D front left
DFR = (q+s.*(NFR-N0)./N0).*NFR; % coefficient D front right
DRL = (q+s.*(NRL-N0)./N0).*NRL; % coefficient D rear left
DRR = (q+s.*(NRR-N0)./N0).*NRR; % coefficient D rear right
% lateral forces
FyFL = DFL.*sin(C.*atan(BF.*alphaF-E.*(BF.*alphaF-atan(BF.*alphaF))));
FyFR = DFR.*sin(C.*atan(BF.*alphaF-E*(BF.*alphaF-atan(BF.*alphaF))));
FyF  = (FyFL + FyFR).*cos(delta);

FyRL = DRL.*sin(C.*atan(BR.*alphaR-E*(BR.*alphaR-atan(BR.*alphaR))));
FyRR = DRR.*sin(C.*atan(BR.*alphaR-E*(BR.*alphaR-atan(BR.*alphaR))));
FyR  = (FyRL + FyRR);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Figures

%%
figure()
subplot(231);
plot(t,delta*180/pi); 
xlabel('time [s]'); ylabel('[deg]'); grid on;
title('steer angle');
subplot(232);
plot(t,ay); 
xlabel('time [s]'); ylabel('[m/s^2]'); grid on;
title('lateral acceleration');
subplot(233);
plot(t,vx*3.6,t,vy*3.6); 
xlabel('time [s]'); ylabel('[km/h]'); grid on; legend('v_x','v_y');
title('speed')
subplot(234);
plot(t,beta*180/pi); 
xlabel('time [s]'); ylabel('[deg]'); grid on;
title('sideslip angle');
subplot(235);
plot(t,psip); 
xlabel('time [s]'); ylabel('[rad/s]'); grid on;
title('yaw rate');
subplot(236);
plot(t,rhop); 
xlabel('time [s]'); ylabel('[rad/s]'); grid on;
title('roll rate');

% Adjust settings for high quality
set(gca, 'FontSize', 10); % Set font size
set(gca, 'LineWidth', 1.2); % Set line width

% Save the plot as a high-quality image
% saveas(gcf, 'sub1.png'); % Save as PNG
% Alternatively, you can use print function
print(gcf, 'sub1.png', '-dpng', '-r500'); % Save as PNG with 300 DPI
%%

figure()
subplot(231);
plot(t,alfa,t,Uy,t,Vr); grid on; legend('angle','wind speed','relative speed')
xlabel('time [s]'); ylabel('[deg], [m/s]');
title('Wind yaw angle and speed')
subplot(232);
plot(t,FyW,t,MxW,t,MzW); grid on;
xlabel('time [s]'); ylabel('[N],[Nm],[Nm]');
legend('Fy','Mx','Mz');
title('Wind force and moments')
subplot(233);
plot(t,NFL/Fza*2,t,NFR/Fza*2,t,NRL/Fzp*2,t,NRR/Fzp*2),
grid,xlabel('Time [s]'),ylabel('[ ]');
title('Load on wheels ratio N/N_{static}')
legend('FL','FR','RL','RR');
subplot(234);
plot(X,Y);
grid on; xlabel('X [m]'),ylabel('Y [m]');
hold on; plot(x_ref,y_ref,'r--'),legend('cog','Ref');xlim([0 max(X)]); grid on;
title('trajectory');
subplot(235);
plot(t,psi*(180/pi)),grid on;
xlabel('Time [s]'),ylabel('[deg]'); grid on;
title('yaw angle');
subplot(236);
plot(t,rho*(180/pi)),grid;
xlabel('Time [s]'),ylabel('[deg]'); grid on;
title('roll angle');
% Adjust settings for high quality
set(gca, 'FontSize', 10); % Set font size
set(gca, 'LineWidth', 1.2); % Set line width

% Save the plot as a high-quality image
% saveas(gcf, 'sub1.png'); % Save as PNG
% Alternatively, you can use print function
print(gcf, 'sub2.png', '-dpng', '-r500'); % Save as PNG with 300 DPI

%%
figure(200)
plot(X,Y); title('vehicle trajectory and orientation')
% Adjust settings for high quality
set(gca, 'FontSize', 10); % Set font size
set(gca, 'LineWidth', 1.2); % Set line width

% Alternatively, you can use print function
print(gcf, 'high_quality_figure1.png', '-dpng', '-r500'); % Save as PNG with 300 DPI
for ii=1:t(end)
    Xg = interp1(t,X,ii);
    Yg = interp1(t,Y,ii);
    Psi = interp1(t,psi,ii);
    Xpg = 5*cos(Psi);
    Ypg = 5*sin(Psi);
    figure(3)
    quiver(Xg,Yg,Xpg,Ypg,'r'); hold on;
%     axis equal; 
    grid on;
end

% Adjust settings for high quality
set(gca, 'FontSize', 10); % Set font size
set(gca, 'LineWidth', 1.2); % Set line width

% Alternatively, you can use print function
print(gcf, 'high_quality_figure.png', '-dpng', '-r500'); % Save as PNG with 300 DPI
return

% break

% figure()
% subplot(131)
% plot(Wind_Coefficients(1,:),Wind_Coefficients(3,:),'*');
% xlabel('\alpha [deg]'); title('C_y');
% subplot(132)
% plot(Wind_Coefficients(1,:),Wind_Coefficients(5,:),'*');
% xlabel('\alpha [deg]'); title('C_{mx}');
% subplot(133)
% plot(Wind_Coefficients(1,:),Wind_Coefficients(4,:),'*');
% xlabel('\alpha [deg]'); title('C_{mz}');
% 
% iu = [7 14 25]';
% v25 = [140 130 115]';
% v30 = [140 125 80]';
% 
% figure()
% plot(v25,iu,'*-',v30,iu,'*-'); grid on;
% legend('U = 25 m/s','U = 30 m/s');
% xlabel('vehicle speed [km/h]');

figure(100)
plot(t,NFL,t,NFR,t,NRL,t,NRR),
grid,xlabel('Time [s]'),ylabel('[N]');
title('Load on wheels')
legend('FL','FR','RL','RR');
% Adjust settings for high quality
set(gca, 'FontSize', 10); % Set font size
set(gca, 'LineWidth', 1.2); % Set line width

% Save the figure as a high-quality image
saveas(gcf, 'high_quality_figure.png'); % Save as PNG
% Alternatively, you can use print function
% print(gcf, 'high_quality_figure.png', '-dpng', '-r300'); % Save as PNG with 300 DPI
%%
results.settings = {road_select, nomefile, V, tfin};
fileID = fopen('Results.txt', 'w');
fprintf(fileID, '%6.2f %12.8f\n', [road_select, nomefile, V, tfin]);
fclose(fileID);