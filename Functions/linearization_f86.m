function linearization = linearization_f86(Zm,Vm,PSI0,alt0,V_0)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

syms z u v w phi theta PSI p q r V CDwave
syms zdot udot vdot wdot phidot thetadot psidot pdot qdot rdot Vdot
syms Vxwind Vywind Vzwind deltaP deltaE deltaA deltaR xCOG
syms z0 u0 v0 w0 phi0 theta0 psi0 p0 q0 r0 V0 deltaP0 deltaE0 deltaA0 deltaR0
syms zdot0 udot0 vdot0 wdot0 phidot0 thetadot0 psidot0 pdot0 qdot0 rdot0 Vdot0

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

%% THRUST AND TSFC MODEL

TSL    = 26289;
Thrust = TSL*(sigma^1.2)*deltaP;
%TSFC0  = 2.56*10^(-5);
%TSFC   = TSFC0*(sigma^0.1);

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

%xcog0 = 2.50; % Gravity center wrt wing apex
%xcogf = 0.48; % Gravity center wrt wing apex
%xcog  = (m0i*xcog0+xcogf*(FM-m0i+m))/m; % Gravity center wrt wing apex
xcog   = 2.50;

%% ROTATION MATRIX

girophi   = [1,0,0;0,cos(phi),sin(phi);0,(-1).*sin(phi),cos(phi)];
girotheta = [cos(theta),0,(-1).*sin(theta);0,1,0;sin(theta),0,cos(theta)];
giropsi   = [cos(PSI),sin(PSI),0;(-1).*sin(PSI),cos(PSI),0;0,0,1];
MBH       = girophi*girotheta*giropsi; % Local horizon axes to body axes
MBHdot    = [(-1).*psidot.*cos(theta).*sin(PSI)+(-1).*thetadot.*cos(PSI).*sin(...
            theta),psidot.*cos(PSI).*cos(theta)+(-1).*thetadot.*sin(PSI).*sin(...
            theta),(-1).*thetadot.*cos(theta);(-1).*psidot.*cos(phi).*cos(...
            PSI)+thetadot.*cos(PSI).*cos(theta).*sin(phi)+phidot.*sin(phi).*sin(...
            PSI)+phidot.*cos(phi).*cos(PSI).*sin(theta)+(-1).*psidot.*sin(phi)...
            .*sin(PSI).*sin(theta),(-1).*phidot.*cos(PSI).*sin(phi)+(-1).*...
            psidot.*cos(phi).*sin(PSI)+thetadot.*cos(theta).*sin(phi).*sin(...
            PSI)+psidot.*cos(PSI).*sin(phi).*sin(theta)+phidot.*cos(phi).*sin(...
            PSI).*sin(theta),phidot.*cos(phi).*cos(theta)+(-1).*thetadot.*sin(...
            phi).*sin(theta);thetadot.*cos(phi).*cos(PSI).*cos(theta)+psidot.*...
            cos(PSI).*sin(phi)+phidot.*cos(phi).*sin(PSI)+(-1).*phidot.*cos(...
            PSI).*sin(phi).*sin(theta)+(-1).*psidot.*cos(phi).*sin(PSI).*sin(...
            theta),(-1).*phidot.*cos(phi).*cos(PSI)+thetadot.*cos(phi).*cos(...
            theta).*sin(PSI)+psidot.*sin(phi).*sin(PSI)+psidot.*cos(phi).*cos(...
            PSI).*sin(theta)+(-1).*phidot.*sin(phi).*sin(PSI).*sin(theta),(-1)...
            .*phidot.*cos(theta).*sin(phi)+(-1).*thetadot.*cos(phi).*sin(theta)];

%% WIND PARAMETERS

uvwwind  = MBH*[Vxwind;Vywind;Vzwind];
uwind    = uvwwind(1,1);
vwind    = uvwwind(2,1);
wwind    = uvwwind(3,1);
% Wind vector derivatives are considered negligible in linearization
uvwwinddot = MBHdot*[Vxwind;Vywind;Vzwind];
uwinddot   = uvwwinddot(1,1);
vwinddot   = uvwwinddot(2,1);
wwinddot   = uvwwinddot(3,1);

%% AERODYNAMIC PARAMETERS

% Expressions of alpha, beta and V
urel     = u-uwind;
vrel     = v-vwind;
wrel     = w-wwind;
ureldot  = udot-uwinddot;
vreldot  = vdot-vwinddot;
wreldot  = wdot-wwinddot;

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
CD = CD0+K_f*CL^2+CDwave;
% Moment coefficient
CMdeltaE   = CLdeltaE*(St/Sw)*(xcog-xACt)/cw;
CMalphaw   = CLalphaw*(xcog-xACw)/cw;
CMalphat   = CLalphat*(1+downstream)*(xcog-xACt)*(St/Sw)/cw;
CMalpha    = CMalphaw+CMalphat;
CMq        = -CLq*(xACt-xcog)/cw;
CMalphadot = -CLalphadot*(xACt-xACw)/cw;
CM         = CMalpha*alpha+CMdeltaE*deltaE+CMq*(cw/(2*V_total))*q+...
             CMalphadot*(cw/(2*V_total))*alphadot;
% Lateral force coefficient
CYbeta    = -0.434283;
CYdeltaA  = 0;
CYdeltaR  = 0.0512434;
CYbetadot = 0;
CYp       = (-0.694841E-1).*(0.175E1.*cos(alpha)+(-0.54E1).*sin(alpha));
CYr       = 0.694841E-1.*(0.54E1.*cos(alpha)+0.175E1.*sin(alpha));
CY        = CYbeta*beta+CYdeltaA*deltaA+CYdeltaR*deltaR+CYbetadot*(bw/(2*V_total))*betadot...
            +CYp*(bw/(2*V_total))*p+CYr*(bw/(2*V_total))*r;
% Roll moment coefficient
CLbeta    = 0.573E2.*((-0.12E-2)+(-0.2925E-2).*(0.12E1+(0.285714E0+( ...
            0.297619E0+((-0.345238E0)+(0.327381E0+(0.102183E2+(0.426587E2+ ...
            0.257937E3.*((-0.3E0)+0.845012E0.*M)).*((-0.7E0)+0.845012E0.*M)).* ...
            ((-0.2E0)+0.845012E0.*M)).*((-0.6E0)+0.845012E0.*M)).*((-0.4E0)+ ...
            0.845012E0.*M)).*((-0.1E0)+0.845012E0.*M)).*((-0.8E0)+0.845012E0.* ...
            M))).*(0.270193E1.*deltaE.*(0.2E1+(0.339474E2+(-0.216673E2).*M.^2) ...
            .^(1/2)).^(-1)+alpha.*(0.298879E2.*(0.2E1+(0.356888E2+( ...
            -0.226273E2).*M.^2).^(1/2)).^(-1)+(2+(0.339474E2+(-0.216673E2).* ...
            M.^2).^(1/2)).^(-1).*(0.355769E1+(-0.123561E2).*(0.2E1+( ...
            0.356888E2+(-0.226273E2).*M.^2).^(1/2)).^(-1))))+0.573E2.*( ...
            0.558941E-3+(1/60).*((-0.153459E-1)+(-0.916732E-2).*(0.1275E1+( ...
            0.392857E0+(0.357143E0+(0.303571E1+((-0.14881E0)+((-0.43186E2)+(( ...
            -0.160122E3)+(0.185708E4+(0.327502E4+0.184705E5.*((-0.1226E0)+ ...
            0.845012E0.*M)).*((-0.7E0)+0.845012E0.*M)).*((-0.3E0)+0.845012E0.* ...
            M)).*((-0.75E0)+0.845012E0.*M)).*((-0.2E0)+0.845012E0.*M)).*(( ...
            -0.6E0)+0.845012E0.*M)).*((-0.4E0)+0.845012E0.*M)).*((-0.1E0)+ ...
            0.845012E0.*M)).*((-0.8E0)+0.845012E0.*M))).*pi)+(-0.347421E-1).*( ...
            0.175E1.*cos(alpha)+(-0.54E1).*sin(alpha));
CLdeltaA  = 0.169744;
CLdeltaR  = 0.453158E-2.*(0.1658E1.*cos(alpha)+(-0.605E1).*sin(alpha));
CLbetadot = 0;
CLp       = (-0.18818E-1)+(-0.355561E0).*(1+(-1).*M.^2).^(1/2);
CLr       = 0.465558E-2+0.35E0.*(1+0.612863E-1.*(0.16383E1+0.475681E1.*(1+( ...
            -0.67101E0).*M.^2).^(1/2)).*(0.327661E1+0.475681E1.*(1+( ...
            -0.67101E0).*M.^2).^(1/2)).^(-1)).^(-1).*(1+0.159594E1.*M.^2.*(1+( ...
            -0.67101E0).*M.^2).^(-1/2).*(0.16383E1+0.475681E1.*(1+(-0.67101E0) ...
            .*M.^2).^(1/2)).^(-1)+0.612863E-1.*(0.16383E1+0.475681E1.*(1+( ...
            -0.67101E0).*M.^2).^(1/2)).*(0.327661E1+0.475681E1.*(1+( ...
            -0.67101E0).*M.^2).^(1/2)).^(-1)).*(0.270193E1.*deltaE.*(0.2E1+( ...
            0.339474E2+(-0.216673E2).*M.^2).^(1/2)).^(-1)+alpha.*(0.298879E2.* ...
            (0.2E1+(0.356888E2+(-0.226273E2).*M.^2).^(1/2)).^(-1)+(2+( ...
            0.339474E2+(-0.216673E2).*M.^2).^(1/2)).^(-1).*(0.355769E1+( ...
            -0.123561E2).*(0.2E1+(0.356888E2+(-0.226273E2).*M.^2).^(1/2)).^( ...
            -1))))+0.614464E-2.*(0.175E1.*cos(alpha)+(-0.54E1).*sin(alpha)).*( ...
            0.54E1.*cos(alpha)+0.175E1.*sin(alpha));
Cl        = CLbeta*beta+CLdeltaA*deltaA+CLdeltaR*deltaR+CLbetadot*(bw/(2*V_total))*betadot...
            +CLp*(bw/(2*V_total))*p+CLr*(bw/(2*V_total))*r;
% Yaw moment coefficient
CNbeta    = (-0.21002E-1)+0.347421E-1.*(0.54E1.*cos(alpha)+0.175E1.*sin( ...
            alpha));
CNdeltaA  = 0.424361E-2.*(0.270193E1.*deltaE.*(0.2E1+(0.339474E2+(-0.216673E2) ...
            .*M.^2).^(1/2)).^(-1)+alpha.*(0.298879E2.*(0.2E1+(0.356888E2+( ...
            -0.226273E2).*M.^2).^(1/2)).^(-1)+(2+(0.339474E2+(-0.216673E2).* ...
            M.^2).^(1/2)).^(-1).*(0.355769E1+(-0.123561E2).*(0.2E1+( ...
            0.356888E2+(-0.226273E2).*M.^2).^(1/2)).^(-1))));
CNdeltaR  = (-0.453158E-2).*(0.605E1.*cos(alpha)+0.1658E1.*sin(alpha));
CNbetadot = 0;
CNp       = (-0.10627E-1).*(1+(-0.67101E0).*M.^2).^(-1/2).*(0.270193E1.* ...
            deltaE.*(0.2E1+(0.339474E2+(-0.216673E2).*M.^2).^(1/2)).^(-1)+ ...
            alpha.*(0.298879E2.*(0.2E1+(0.356888E2+(-0.226273E2).*M.^2).^(1/2) ...
            ).^(-1)+(2+(0.339474E2+(-0.216673E2).*M.^2).^(1/2)).^(-1).*( ...
            0.355769E1+(-0.123561E2).*(0.2E1+(0.356888E2+(-0.226273E2).*M.^2) ...
            .^(1/2)).^(-1))))+0.614464E-2.*((-0.175E1)+0.175E1.*cos(alpha)+( ...
            -0.54E1).*sin(alpha)).*(0.54E1.*cos(alpha)+0.175E1.*sin(alpha))+ ...
            0.353005E0.*(1+(-1).*M.^2).^(1/2).*tan(alpha)+((-0.18818E-1)+( ...
            -0.355561E0).*(1+(-1).*M.^2).^(1/2)).*tan(alpha);
CNr       = (-0.5376E-2)+(-0.5E-1).*(0.270193E1.*deltaE.*(0.2E1+(0.339474E2+ ...
            (-0.216673E2).*M.^2).^(1/2)).^(-1)+alpha.*(0.298879E2.*(0.2E1+( ...
            0.356888E2+(-0.226273E2).*M.^2).^(1/2)).^(-1)+(2+(0.339474E2+( ...
            -0.216673E2).*M.^2).^(1/2)).^(-1).*(0.355769E1+(-0.123561E2).*( ...
            0.2E1+(0.356888E2+(-0.226273E2).*M.^2).^(1/2)).^(-1)))).^2+( ...
            -0.614464E-2).*(0.54E1.*cos(alpha)+0.175E1.*sin(alpha)).^2;
CN        = CNbeta*beta+CNdeltaA*deltaA+CNdeltaR*deltaR+CNbetadot*(bw/(2*V_total))*betadot...
            +CNp*(bw/(2*V_total))*p+CNr*(bw/(2*V_total))*r;

% Neutral point: evaluated at velocities 150, 200 and 250 m/s for z = 0

CMalphawxnp   = CLalphaw*(xCOG-xACw)/cw;
CMalphatxnp   = CLalphat*(1+downstream)*(xCOG-xACt)*(St/Sw)/cw;
CMalphaxnp    = CMalphawxnp+CMalphatxnp;
xnpV150       = double(solve(subs(CMalphaxnp,[u v w Vxwind Vywind Vzwind z],...
                [200 0 0 0 0 0 0])));
xnpV200       = double(solve(subs(CMalphaxnp,[u v w Vxwind Vywind Vzwind z],...
                [200 0 0 0 0 0 0])));
xnpV250       = double(solve(subs(CMalphaxnp,[u v w Vxwind Vywind Vzwind z],...
                [250 0 0 0 0 0 0])));
xnp           = [xnpV150 xnpV200 xnpV250];

%% AERODYNAMIC FORCES

Cxyz = [cos(alpha),0,(-1).*sin(alpha);0,1,0;sin(alpha),0,cos(alpha)]*...
       [-CD;CY;-CL];
Cx   = Cxyz(1,1);
Cy   = Cxyz(2,1);
Cz   = Cxyz(3,1);

FxA  = (1/2)*rho*Sw*V_total^2*Cx;
FyA  = (1/2)*rho*Sw*V_total^2*Cy;
FzA  = (1/2)*rho*Sw*V_total^2*Cz;
Lo   = (1/2)*rho*Sw*V_total^2*bw*Cl;
Mo   = (1/2)*rho*Sw*V_total^2*cw*CM;
No   = (1/2)*rho*Sw*V_total^2*bw*CN;

%% BRYAN EQUATIONS

g         = 9.81;
g_b       = MBH*[0;0;g];
V_vect    = [u;v;w]; 
V_vecdot  = [udot;vdot;wdot];
omega     = [p;q;r]; % Angular velocity
omegadot  = [pdot;qdot;rdot];
V_absdot  = transpose(V_vecdot)+cross(transpose(omega),transpose(V_vect));
Fx        = FxA+Thrust+m*g_b(1,1);
Fy        = FyA+m*g_b(2,1);
Fz        = FzA+m*g_b(3,1);
%%% Mass is considered to be constant %%%
ec1       = Fx-m*V_absdot(1,1);
ec2       = Fy-m*V_absdot(1,2);
ec3       = Fz-m*V_absdot(1,3);

% Angular momentum (inertia moments are considered constants) %
Imatrix   = [Ixx,-Ixy,-Ixz;-Iyx,Iyy,-Iyz;-Izx,-Izy,Izz];
H_vect    = Imatrix*omega;
H_vectdot = Imatrix*omegadot;
H_absdot  = transpose(H_vectdot)+cross(transpose(omega),transpose(H_vect));
ec4       = H_absdot(1,1)-Lo;
ec5       = H_absdot(1,2)-Mo;
ec6       = H_absdot(1,3)-No;

pqr       = [phidot;0;0]+girophi*([0;thetadot;0]+girotheta*[0;0;psidot]);
ec7       = p-pqr(1,1);
ec8       = q-pqr(2,1);
ec9       = r-pqr(3,1);

dxyz      = transpose(MBH)*V_vect;

ec10      = -zdot-dxyz(3,1);

ec11      = -Vdot+(1/2).*(u.^2+v.^2+w.^2).^(-1/2).*(2.*u.*udot+2.*v.*vdot+2.*w.*wdot);

%ec12      = mdot+TSFC*Thrust;

% ec13      = -rumbdot+psidot+(1+(v.*cos(phi)+(-1).*w.*sin(phi)).^2.*(u.* ...
%             cos(theta)+w.*cos(phi).*sin(theta)+v.*sin(phi).*sin(theta)).^(-2)) ...
%             .^(-1).*((vdot.*cos(phi)+(-1).*phidot.*w.*cos(phi)+(-1).*phidot.* ...
%             v.*sin(phi)+(-1).*wdot.*sin(phi)).*(u.*cos(theta)+w.*cos(phi).* ...
%             sin(theta)+v.*sin(phi).*sin(theta)).^(-1)+(-1).*(v.*cos(phi)+(-1) ...
%             .*w.*sin(phi)).*(u.*cos(theta)+w.*cos(phi).*sin(theta)+v.*sin(phi) ...
%             .*sin(theta)).^(-2).*(udot.*cos(theta)+thetadot.*w.*cos(phi).*cos( ...
%             theta)+thetadot.*v.*cos(theta).*sin(phi)+(-1).*thetadot.*u.*sin( ...
%             theta)+phidot.*v.*cos(phi).*sin(theta)+wdot.*cos(phi).*sin(theta)+ ...
%             vdot.*sin(phi).*sin(theta)+(-1).*phidot.*w.*sin(phi).*sin(theta)));

%% STATE-SPACE LINEARIZATION

%%% System and control matrices %%%

variables   = [z u v w phi theta PSI p q r V zdot udot vdot wdot ...
              phidot thetadot psidot pdot qdot rdot Vdot deltaP ...
              deltaE deltaA deltaR Vxwind Vywind Vzwind CDwave];
values0     = [z0 u0 v0 w0 phi0 theta0 psi0 p0 q0 r0 V0 zdot0 udot0 ...
              vdot0 wdot0 phidot0 thetadot0 psidot0 pdot0 qdot0 rdot0 ...
              Vdot0 deltaP0 deltaE0 deltaA0 deltaR0 0 0 0 0];

% Matrix c1 (11x30) contains the coefficients of all derivatives as a 
% function of values0. 
% 0 = c1(1:11,1:11)*x+c1(1:11,12:22)*xdot+c1(1:11,23:26)*u+c1(1:12,27:29)*d+c1(1:11,30)*CDwave

for i = 1:11
    ecnumber = eval(append("ec",string(i)));
    for j = 1:length(variables)
        c1(i,j) = subs(diff(ecnumber,variables(j)),variables,values0);
    end  
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

linearization.c1 = c1;
linearization.values0 = values0;
linearization.xnp     = xnp;
linearization.z0      = Zm;
linearization.z_0     = alt0;
linearization.V0      = Vm;
linearization.V_0     = V_0;
linearization.psi0    = PSI0;

save('linearization.mat','linearization')

clear all
close all

load('linearization.mat')

var = optimvar('var',5);

global z psi V
z   = linearization.z0;
psi = linearization.psi0;
V   = linearization.V0;

options            = optimoptions('fsolve','Display','iter','FunctionTolerance',1e-20);
trimmed_point.sol  = fsolve(@trimmed_point,[V 0.025*V 1*pi/180 0.4 -3*pi/180],options);

linearization.trimmed_point = [z trimmed_point.sol(1) 0 trimmed_point.sol(2) 0 ...
                              trimmed_point.sol(3) psi 0 0 0 V 0 0 0 0 0 0 0 0 0 ...
                              0 0 trimmed_point.sol(4) trimmed_point.sol(5) 0 0];

save('linearization.mat','linearization')

clear all
close all

load('linearization.mat')

var = optimvar('var',5);

global z psi V
z   = linearization.z_0;
psi = linearization.psi0;
V   = linearization.V_0;

options            = optimoptions('fsolve','Display','iter','FunctionTolerance',1e-20);
initial  = fsolve(@trimmed_point,[V 0.025*V 1*pi/180 0.4 -3*pi/180],options);

linearization.initial = [z initial(1) 0 initial(2) 0 ...
                         initial(3) psi 0 0 0 V 0 0 0 0 0 0 0 0 0 ...
                         0 0 initial(4) initial(5) 0 0];

end
