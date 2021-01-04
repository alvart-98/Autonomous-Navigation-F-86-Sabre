function ecs = trimmed_point(var)

% Obtention of the trimmed conditions from determinated values of variables
global z psi V
x = 0;
y = 0;
u = var(1);
v = 0;
w = var(2);
phi = 0;
theta = var(3);
p = 0;
q = 0;
r = 0;
xdot = V*cos(psi);
ydot = V*sin(psi);
zdot = 0;
udot = 0;
vdot = 0;
wdot = 0;
phidot = 0;
thetadot = 0;
psidot = 0;
pdot = 0;
qdot = 0;
rdot = 0;
deltaP = var(4);
deltaE = var(5);
deltaA = 0;
deltaR = 0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% ATMOSPHERIC PARAMETERS

rho_0 = 1.225;
mu_0  = 1.78941*10^(-5);
T_0   = 288.15;
p_0   = 101325;
gamma = 1.4;
R     = 286.9;

P     = p_0*(1+(-0.2257E-4).*z).^0.5256E1;
rho   = rho_0*(1+(-0.2257E-4).*z).^0.4256E1;
T     = T_0*(1+(-0.2257E-4).*z);
mu    = 0.1458E-5.*T.^(3/2).*(0.1104E3+T).^(-1);
a     = sqrt(gamma*R*T);

sigma = rho/rho_0;
thetaT= T/T_0;

%% THRUST MODEL

TSL    = 26289;
Thrust = TSL*(sigma^1.2)*deltaP;

%% GEOMETRICAL PARAMETERS

fttomt = 0.3048; %Ft to mts

cwr     = 10.3*fttomt; %Wing
cwt     = 5.3*fttomt;
bw      = 37.1*fttomt;
Sw      = 26.882;
lambdaw = cwt/cwr;
cw      = (2/3)*cwr*(1+lambdaw+lambdaw^2)/(1+lambdaw);
ARw     = bw^2/Sw;
etaMACw = (1/3)*(1+2*lambdaw)/(1+lambdaw);
Yw      = bw/2*etaMACw;
Gammaw  = 3*pi/180;
LbdaLEw = 37.5094;

ctr     = 3.8*fttomt; %Horizontal stabilizer
ctt     = 1.7*fttomt;
bt      = 12.8*fttomt;
St      = 3.27;
lambdat = ctt/ctr;
ct      = (2/3)*ctr*(1+lambdat+lambdat^2)/(1+lambdat);
ARt     = bt^2/St;
etaMACt = (1/3)*(1+2*lambdat)/(1+lambdat);
Yt      = bt/2*etaMACt;
Gammat  = 10*pi/180;
LbdaLEt = 38.0337;

cvr     = 6.4*fttomt; %Vertical stabilizer
cvt     = 2.3*fttomt;
hvs     = 7.5*fttomt;
Sv      = 3.0307;
lambdav = cvt/cvr;
cv      = (2/3)*cvr*(1+lambdav+lambdav^2)/(1+lambdav);
ARv     = hvs^2/Sv;

Lfuselage = 37.5*fttomt;
Sfuselage = 38.0806;

hiatus   = 4.00543;
alturawt = 1.21446;

xACw     = Yw*tand(LbdaLEw)+0.25*cw;
xACt     = Yt*tand(LbdaLEt)+0.25*ct+hiatus+cwr;

%% MASS AND INERTIA MOMENT

m    = 7578.21;
FM   = 1822.08;
Ixx  = (m-839.18)*(bw/2)^2*0.322^2+3703.73;
Iyy  = (m-839.18)*(Lfuselage/2)^2*0.346^2+87.18;
Izz  = (m-839.18)*((bw+Lfuselage)/2)^2*0.464^2+87.18;
Ixy  = 0;
Iyx  = 0;
Ixz  = 0;
Izx  = 0;
Iyz  = 0;
Izy  = 0;
xcog   = 2.50; % Gravity center wrt wing apex

%% ROTATION MATRIX

girophi   = [1,0,0;0,cos(phi),sin(phi);0,(-1).*sin(phi),cos(phi)];
girotheta = [cos(theta),0,(-1).*sin(theta);0,1,0;sin(theta),0,cos(theta)];
giropsi   = [cos(psi),sin(psi),0;-sin(psi),cos(psi),0;0,0,1];
MBH       = girophi*girotheta*giropsi; % Local horizon axes to body axes

%% AERODYNAMIC PARAMETERS

% Expressions of alpha, beta and V
urel     = u;
vrel     = v;
wrel     = w;
ureldot  = udot;
vreldot  = vdot;
wreldot  = wdot;

V_total  = sqrt(urel^2+vrel^2+wrel^2);
alpha    = atan(wrel/urel);
beta     = asin(vrel/V_total);
V_dot    = (urel*ureldot+vrel*vreldot+wrel*wreldot)/V_total;
alphadot = (urel*wreldot-wrel*ureldot)/(urel^2+wrel^2);
betadot  = (vreldot*V_total-vrel*V_dot)/(V_total^2*sqrt(1-(vrel/V_total)^2));

M        = V_total/a;

% dedalpha & CD parameters
downstream = (-0.347306E1)*(0.2E1+(0.356888E2+(-0.226273E2)*M^2)^(1/2))^(-1);
oswald_f   = 0.780948; % Oswald factor
K_f        = 1/(pi*ARw*oswald_f); % Factor K of induced drag
CD0        = 0.0128115; % Parasitic drag coefficient

% Lift coefficient
CLdeltaE   = 0.222121E2.*(0.2E1+(0.339474E2+(-0.216673E2).*M.^2).^(1/2)).^(-1);
CLalphaw   = 0.298879E2.*(2+(4+0.226273E2.*(0.10001E1+(-1).*M.^2)).^(1/2)).^(-1);
CLalphat   = 0.29247E2.*(2+(4+0.216673E2.*(0.100009E1+(-1).*M.^2)).^(1/2)).^(-1);
CLalpha    = CLalphaw+CLalphat*(1+downstream)*St/Sw;
CLq        = 2*CLalphat*(St/Sw)*(xACt-xcog)/cw;
CLalphadot = -2*CLalphat*(St/Sw)*(xACt-xcog)*downstream/cw;
CL         = CLalpha*alpha+CLdeltaE*(St/Sw)*deltaE+CLq*(cw/(2*V_total))*q+...
             CLalphadot*(cw/(2*V_total))*alphadot;
% Drag coefficient
CD = CD0+K_f*CL^2;
% Moment coefficient
CMdeltaE   = CLdeltaE*(St/Sw)*(xcog-xACt)/cw;
CMalphaw   = CLalphaw*(xcog-xACw)/cw;
CMalphat   = CLalphat*(1+downstream)*(xcog-xACt)*(St/Sw)/cw;
CMalpha    = CMalphaw+CMalphat;
CMq        = -CLq*(xACt-xcog)/cw;
CMalphadot = -CLalphadot*(xACt-xACw)/cw;
CM         = CMalpha*alpha+CMdeltaE*deltaE+CMq*(cw/(2*V_total))*q+...
             CMalphadot*(cw/(2*V_total))*alphadot;

%% AERODYNAMIC FORCES

Cxyz = [cos(alpha),(-1).*sin(alpha);sin(alpha),cos(alpha)]*...
       [-CD;-CL];
Cx   = Cxyz(1,1);
Cz   = Cxyz(2,1);

FxA  = (1/2)*rho*Sw*V_total^2*Cx;
FzA  = (1/2)*rho*Sw*V_total^2*Cz;

%% BRYAN EQUATIONS

g         = 9.81;
g_b       = MBH*[0;0;g];
V_vect    = [u;v;w]; 
V_vecdot  = [udot;vdot;wdot];
omega     = [p;q;r]; % Angular velocity
omegadot  = [pdot;qdot;rdot];
V_absdot  = transpose(V_vecdot)+cross(transpose(omega),transpose(V_vect));
Fx        = FxA+Thrust+m*g_b(1,1);
Fz        = FzA+m*g_b(3,1);
%%% Mass is considered to be constant %%%
ec1       = Fx-m*V_absdot(1,1);
ec3       = Fz-m*V_absdot(1,3);

% Angular momentum (inertia moments are considered constants) %
ec5       = CM;

dxyz      = transpose(MBH)*V_vect;
ec10      = xdot-dxyz(1,1);
ec12      = -zdot-dxyz(3,1);

%% TRIMMED CONDITIONS

ecs = [ec1,ec3,ec5,ec10,ec12];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
