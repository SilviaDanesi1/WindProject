function xp = Truck_rollover_Equations(t,x)

global g dt
global m Jz Jx Fza Fzp
global a b c zR zG xP zP
global c_roA c_roP k_roA k_roP dNA dNP
global kp kd ki L x_ref y_ref ErrInt
% global profili_vento t_Wind spazio_profili
% global fid
global indice xp_prec iii xpp
global delta
global vx
global BF BR C E s q N0

vy   = x(1);
PSIp = x(2);
RHOp = x(3);
X    = x(4);
Y    = x(5);
PSI  = x(6);
RHO  = x(7);

Xp = vx*cos(PSI) - vy*sin(PSI);
Yp = vx*sin(PSI) + vy*cos(PSI);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% driver steering angle
Y_ref  = interp1(x_ref, y_ref, X + L*cos(PSI));
Y_oss  = Y + L*sin(PSI);
Yp_oss = Yp + PSIp*L*cos(PSI);
ErrInt = ErrInt + ( Y_ref - Y_oss)*dt;
d      = kp*( Y_ref - Y_oss) + kd*( 0 - Yp_oss) + ki*ErrInt; % rettilineo
d      = d - kp*PSI - kd*PSIp;
% steering angle saturation to avoid eccessive and no physical value
if d > (45*pi/180) 
    d = 45*pi/180;
elseif d < (-15*pi/180)
    d = -45*pi/180;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Aerodynamic forces calculation
[FyW,MxW,MzW] = f_aero(Xp,Yp,PSI,X,Y,t);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% slip angle for contact tyre forces
alphaF = d - atan((vy + PSIp*a)/vx);
alphaR = - atan((vy-PSIp*b)/vx);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Pacejka Tyre force
% load transfer
MA  = k_roA*RHO + c_roA*RHOp;    % roll moment due to roll stiffness and damping
MP  = k_roP*RHO + c_roP*RHOp;
dNa = MA / c; % vertical load transfer front axle
dNp = MP / c; % vertical load transfer rear axle
% load on tyres
NFL = Fza/2 - dNa; % normal load front left
NFR = Fza/2 + dNa; % normal load front right
NRL = Fzp/2 - dNp; % normal load rear left
NRR = Fzp/2 + dNp; % normal load rear right
if ((NFL < 0 || NFR < 0) || NRL < 0) || NRR < 0 
    t
    x
    NFL
    NFR
    NRL
    NRR
    warning(['!! ROLLOVER !! @ t=',num2str(t)]);
end
% pacejka D coefficient
DFL = (q+s*abs(NFL-N0)/N0)*NFL; % coefficient D front left
DFR = (q+s*abs(NFR-N0)/N0)*NFR; % coefficient D front right
DRL = (q+s*abs(NRL-N0)/N0)*NRL; % coefficient D rear left
DRR = (q+s*abs(NRR-N0)/N0)*NRR; % coefficient D rear right
% lateral forces
FyFL = DFL*sin(C*atan(BF*alphaF-E*(BF*alphaF-atan(BF*alphaF))));
FyFR = DFR*sin(C*atan(BF*alphaF-E*(BF*alphaF-atan(BF*alphaF))));
FyF  = (FyFL + FyFR)*cos(d);

FyRL = DRL*sin(C*atan(BR*alphaR-E*(BR*alphaR-atan(BR*alphaR))));
FyRR = DRR*sin(C*atan(BR*alphaR-E*(BR*alphaR-atan(BR*alphaR))));
FyR  = (FyRL + FyRR);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% equation of motion
ay      = 1/m * ( FyF + FyR + FyW);
PSIpp   = 1/Jz* ( FyF*a - FyR*b +  FyW*xP + MzW); %yaw
RHOpp   = 1/Jx* ( m*(zG-zR)*(ay + g*sin(RHO)) - MA - MP + MxW -FyW*(zP-zR) ); %% roll
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% state vector
%       1       2       3       4       5       6       7
% x  = [vy      psip    rhop    X       Y       psi     rho];
% xp = [vyp     psipp   rhopp   Xp      Yp      psip    rhop]
% vy = ay - vx*psip
% Xp = V*cos(beta+psi);
% Yp = V*sin(beta+psi);
xp(1) = ay - vx*PSIp;
xp(2) = PSIpp;
xp(3) = RHOpp;
xp(4) = Xp;
xp(5) = Yp;
xp(6) = PSIp;
xp(7) = RHOp;

indice = indice+1;

if indice == 6
    indice  = 0;
    xp_prec = xp;
    xpp(iii,:) = xp;
    delta(iii,1) = d;
    dNA(iii,1) = dNa;
    dNP(iii,1) = dNp;
    iii = iii+1;
end