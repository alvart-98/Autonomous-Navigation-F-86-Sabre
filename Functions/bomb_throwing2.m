function prmts = bomb_throwing2(path_ref,Vwind,Xelev,Yelev,Zelev,T)

Zimpact = Zelev(end,end);

theta = path_ref(7)*pi/180;
psi   = path_ref(8)*pi/180;
u = path_ref(4);
v = path_ref(5);
w = path_ref(6);
q = path_ref(9)*pi/180;
r = path_ref(10)*pi/180;
x = path_ref(1);
y = path_ref(2);
z = path_ref(3);

i = 1;

prmts = [0 u v w theta psi q r x y z];

dt = T;

%% GEOMETRICAL PARAMETERS

fttomt  = 0.3048;  % Ft to mts
intomt  = 0.0254;  % Inches to mts
psftopa = 4.44822/fttomt^2; % psf to Pascals

Lb  = 2.134;
ln  = 0.508;
db  = 0.409;
dbb = 0.102;
Sref= pi*db^2/4;
Sbody = 940.315*intomt^2;

cbr     = 26.907*intomt; %Wing
cbt     = 8.504*intomt;
Sb      = 396.607*intomt^2;
bb      = 22.40*intomt;
lambdab = cbt/cbr;
cb      = (2/3)*cbr*(1+lambdab+lambdab^2)/(1+lambdab);
ARb     = bb^2/Sb;
LbdaLEb = 58.676;
Lbdac4b = 50.942;
etaMACb = (1/3)*(1+2*lambdab)/(1+lambdab);
Yb      = bb/2*etaMACb;

%% MASS AND INERTIA MOMENT

m    = 362.44;
Ixx  = 6.944;
Iyy  = 40.703;
Izz  = 40.703;

xcog   = 0.784;

%% Data 

g = 9.81;

rho_0 = 1.225;
mu_0  = 1.78941*10^(-5);
T_0   = 288.15;
p_0   = 101325;
gamma = 1.4;
R     = 286.9;

%% Induced drag

oswald_f = (1-0.045*ARb^0.68)*(1-0.227*(Lbdac4b*pi/180)^1.615); % Oswald factor
K_f      = 1/(pi*ARb*oswald_f); % Factor K of induced drag

%% Loop While

while z > Zimpact

%% ATMOSPHERIC PARAMETERS

P     = p_0*(1+(-0.2257E-4).*z).^0.5256E1;
rho   = rho_0*(1+(-0.2257E-4).*z).^0.4256E1;
T     = T_0*(1+(-0.2257E-4).*z);
mu    = 0.1458E-5.*T.^(3/2).*(0.1104E3+T).^(-1);
a     = sqrt(gamma*R*T);

sigma = rho/rho_0;
thetaT= T/T_0;

%% AERODYNAMIC PARAMETERS

girotheta = [cos(theta),0,(-1).*sin(theta);0,1,0;sin(theta),0,cos(theta)];
giropsi   = [cos(psi),sin(psi),0;(-1).*sin(psi),cos(psi),0;0,0,1];
MBH       = girotheta*giropsi; % Local horizon axes to body axes

Vxwind   = Vwind(i,1);
Vywind   = Vwind(i,2);
Vzwind   = Vwind(i,3);

uvwwind  = MBH*[Vxwind;Vywind;Vzwind];
uwind    = uvwwind(1,1);
vwind    = uvwwind(2,1);
wwind    = uvwwind(3,1);

% Expressions of alpha, beta and V
urel     = u-uwind;
vrel     = v-vwind;
wrel     = w-wwind;

V_total  = sqrt(urel^2+vrel^2+wrel^2);
alpha    = atan(wrel/urel);
beta     = asin(vrel/V_total);

M        = V_total/a;

% Aerodynamic centers

xACb = ln*(0.63*(1-(sin(alpha))^2)+0.5*(Lb/ln)*(sin(alpha))^2);

% CD parameters

Q        = 0.5*rho*V_total^2;
if M <= 1
    CD0base  = 0.12+0.13*M^2;
    CD0fricb = 0.053*(Lb/db)*(M/(Q*Lb*psftopa*fttomt))^0.2;
    CD0fricf = 4*(0.0133*(M/(Q*cb*psftopa*fttomt))^0.2)*(2*Sb/Sref);
    CD0      = CD0base+CD0fricb+CD0fricf;
    xACf     = cb*0.25+Yb*tand(LbdaLEb)+Lb-cbr;
else 
    CD0base  = 0.25/M;
    CD0waveb = (1.568+1.834/M^2)*(atan(0.5/(ln/db)))^1.69;
    CD0fricb = 0.053*(Lb/db)*(M/(Q*Lb*psftopa*fttomt))^0.2;
    CD0fricf = 4*(0.0133*(M/(Q*cb*psftopa*fttomt))^0.2)*(2*Sb/Sref);
    CD0      = CD0base+CD0waveb+CD0fricb+CD0fricf;
    xACf     = cb*(ARb*sqrt(M^2-1)-0.67)/(2*ARb*sqrt(M^2-1)-1)+Yb*tand(LbdaLEb)+Lb-cbr;
end

% Lift coefficient
if alpha == 0
    CLalphab = 0;
    CLalphaf = 0;
else
    CLalphab = (abs(sin(2*alpha)*cos(alpha/2))+2*(Lb/db)*(sin(alpha))^2)/abs(alpha);
    CLalphaf = ((pi*ARb/2)*abs(sin(alpha)*cos(alpha))+2*(sin(alpha))^2)*(Sb/Sref)/abs(alpha);
end
CLqb     = 2*CLalphab*(Sbody/Sref)*(xACb-xcog)/cb;
CLqf     = 2*CLalphaf*(Sb/Sref)*(xACf-xcog)/cb;
CL       = (CLalphab+CLalphaf)*alpha+(CLqb+CLqf)*(cb/(2*V_total))*q;
% Lateral force coefficient
if beta == 0
    CYbetab = 0;
    CYbetaf = 0;
else
    CYbetab = -(abs(sin(2*beta)*cos(beta/2))+2*(Lb/db)*(sin(beta))^2)/abs(beta);
    CYbetaf = -((pi*ARb/2)*abs(sin(beta)*cos(beta))+2*(sin(beta))^2)*(Sb/Sref)/abs(beta);
end
CYrb    = -2*CYbetab*(Sbody/Sref)*(xACb-xcog)/cb;
CYrf    = -2*CYbetaf*(Sb/Sref)*(xACf-xcog)/cb;
CY      = (CYbetab+CYbetaf)*beta+(CYrb+CYrf)*(cb/(2*V_total))*r;
% Drag coefficient
CD = CD0+K_f*(CL^2+CY^2);
% Moment coefficient
CMalphab   = CLalphab*(xcog-xACb)/cb;
CMalphaf   = CLalphaf*(xcog-xACf)/cb;
CMqb       = -CLqb*(xACb-xcog)/cb;
CMqf       = -CLqf*(xACf-xcog)/cb;
CM         = (CMalphab+CMalphaf)*alpha+(CMqb+CMqf)*(cb/(2*V_total))*q;
% Roll moment coefficient
% Cl        = 0;
% Yaw moment coefficient
CNbetab   = CYbetab*(xcog-xACb)/cb;
CNbetaf   = CYbetaf*(xcog-xACf)/cb;
CNrb      = -CYrb*(xACb-xcog)/cb;
CNrf      = -CYrf*(xACf-xcog)/cb;
CN        = (CNbetab+CNbetaf)*beta+(CNrb+CNrf)*(cb/(2*V_total))*r;

%% AERODYNAMIC FORCES

Cxyz = [cos(alpha),0,(-1).*sin(alpha);0,1,0;sin(alpha),0,cos(alpha)]*...
       [-CD;CY;-CL];
Cx   = Cxyz(1,1);
Cy   = Cxyz(2,1);
Cz   = Cxyz(3,1);

FxA  = (1/2)*rho*Sref*V_total^2*Cx;
FyA  = (1/2)*rho*Sref*V_total^2*Cy;
FzA  = (1/2)*rho*Sref*V_total^2*Cz;
Mo   = (1/2)*rho*Sref*V_total^2*cb*CM;
No   = (1/2)*rho*Sref*V_total^2*cb*CN;

%% BRYAN EQUATIONS

%%% Mass is considered to be constant %%%
udot      = r*v-q*w+FxA/m-g*sin(theta);
vdot      = -r*u+FyA/m;
wdot      = q*u+FzA/m+g*cos(theta);

% Angular momentum (inertia moments are considered constants) %
qdot      = Mo/Iyy;
rdot      = No/Izz;

thetadot = q;
psidot   = r/cos(theta);

xdot = u*cos(psi)*cos(theta)-v*sin(psi)+w*sin(theta)*cos(psi);
ydot = u*cos(theta)*sin(psi)+v*cos(psi)+w*sin(theta)*sin(psi);
zdot = u*sin(theta)-w*cos(theta);

%% VECTORS DEFINITION

u     = real(u+udot*dt);
v     = real(v+vdot*dt);
w     = real(w+wdot*dt);
theta = real(theta+thetadot*dt);
psi   = real(psi+psidot*dt);
q     = real(q+qdot*dt);
r     = real(r+rdot*dt);
x     = real(x+xdot*dt);
y     = real(y+ydot*dt);
z     = real(z+zdot*dt);

prmts = [prmts;[prmts(end,1)+dt u v w theta psi q r x y z]];

if x <= Xelev(end)
    err1 = abs(Xelev-x);
    err1s  = sort(err1);
    mine11 = find(round(err1,6)==round(err1s(1),6));
    mine12 = find(round(err1,6)==round(err1s(2),6));
end

if y <= Yelev(end)
    err2 = abs(Yelev-y);
    err2s  = sort(err2);
    mine21 = find(round(err2,6)==round(err2s(1),6));
    mine22 = find(round(err2,6)==round(err2s(2),6));
end

if (x > Xelev(end))||(y > Yelev(end))
    Zimpact = Zelev(end,end);
else
    Zvect   = [Zelev(mine21,mine11) Zelev(mine22,mine11);Zelev(mine21,mine12) Zelev(mine22,mine12)];
    Zimpact = (1/((Xelev(mine12)-Xelev(mine11))*(Yelev(mine22)-Yelev(mine21))))...
        *([Xelev(mine12)-x x-Xelev(mine11)]*Zvect*[Yelev(mine22)-y;y-Yelev(mine21)]);
%     A11 = Zelev(mine21,mine11);
%     A21 = Zelev(mine21,mine12)-Zelev(mine21,mine11);
%     A12 = Zelev(mine22,mine11)-Zelev(mine21,mine11);
%     A22 = Zelev(mine22,mine12)+Zelev(mine21,mine11)-Zelev(mine21,mine12)-Zelev(mine22,mine11);
%     Zimpact = A11+A21*x+A12*y+A22*x*y;
end
i = i+1;
end

end