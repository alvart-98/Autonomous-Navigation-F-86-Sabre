function matrices = matrices_generator(linearization,simtime,T,WIND)

% Calculation of control matrices and wind data

load('linearization.mat')

syms z0 u0 v0 w0 phi0 theta0 psi0 p0 q0 r0 V0 deltaP0 deltaE0 deltaA0 deltaR0
syms zdot0 udot0 vdot0 wdot0 phidot0 thetadot0 psidot0 pdot0 qdot0 rdot0 Vdot0

% x: z-1 | phi  -5 | q   -9  ||  u: deltaP-1  ||  perturbation (d): Vxwind-1
%    u-2 | theta-6 | r   -10 ||     deltaE-2  ||                    Vywind-2
%    v-3 | psi  -7 | V   -11 ||     deltaA-3  ||                    Vzwind-3
%    w-4 | p    -8 |         ||     deltaR-4  ||                    CDwave

%% CALCULATION OF STATE-SPACE MATRICES AND COEFFICIENTS

v_0  = [z0 u0 v0 w0 phi0 theta0 psi0 p0 q0 r0 V0 zdot0 udot0 vdot0 ...
       wdot0 phidot0 thetadot0 psidot0 pdot0 qdot0 rdot0 Vdot0 ...
       deltaP0 deltaE0 deltaA0 deltaR0];
v_0trim = linearization.trimmed_point;
v_0i    = linearization.initial;

% Matrix c1 (11x30) contains the coefficients of all derivatives as a 
% function of values0. 0 = c1(1:11,1:11)*x+c1(1:11,12:22)*xdot+
% c1(1:11,23:26)*u+c1(1:12,27:29)*d+c1(1:11,30)*CDwave 

coef_matrix = double(subs(linearization.c1,v_0,v_0trim));

%%% xdot = A*x+B*u+Ad*d %%%
%%% y    = C*x+D*u+Cd*d %%%

inv_xdot = inv(coef_matrix(1:11,12:22));

% Simplified matrices: global
A     = -inv_xdot*coef_matrix(1:11,1:11); 
B     = -inv_xdot*coef_matrix(1:11,23:26);
C     = eye(11);
Cmpc  = [C(1,:);C(5,:);C(7,:);C(11,:)];
D     = zeros(11,4);
Dmpc  = zeros(4,8);
Ad    = -inv_xdot*coef_matrix(1:11,27:29);
Awave = -inv_xdot*coef_matrix(1:11,30);
B     = [B Ad Awave];

%% SAVING OF MATRICES AND PARAMETERS USED IN SIMULINK FILE

matrices.v_0trim  = v_0trim;
matrices.v_0i     = v_0i;
    
matrices.A     = A; 
matrices.B     = B;
matrices.C     = C;
matrices.D     = D;
matrices.Cmpc  = Cmpc;
matrices.Dmpc  = Dmpc;

% Simulation 
matrices.simtime = simtime;
matrices.T       = T;
tiempo           = transpose([0:T:simtime]);

wind = [];
for i = 1:size(tiempo,1)

V    = min(max(WIND(10)*randn+WIND(1),WIND(5)),WIND(4));
psi  = min(max(WIND(11)*randn+WIND(2),WIND(7)),WIND(6));
elev = min(max(WIND(12)*randn+WIND(3),WIND(9)),WIND(8));

Vxwind = V*cosd(elev)*cosd(psi);
Vywind = V*cosd(elev)*sind(psi);
Vzwind = -V*sind(elev);

wind = [wind;[tiempo(i,1) Vxwind Vywind Vzwind]];

end

matrices.wind = wind;

% Max and min values for control actions
radtodeg           = 180/pi;
matrices.radtodeg  = radtodeg;
matrices.maxdeltaP = 1;
matrices.mindeltaP = 0.1;
matrices.maxdeltaE = 2.83/radtodeg;
matrices.mindeltaE = -21/radtodeg;
matrices.maxdeltaA = 15/radtodeg;
matrices.mindeltaA = -15/radtodeg;
matrices.maxdeltaR = 25.5/radtodeg;
matrices.mindeltaR = -25.5/radtodeg;

matrices.maxdeltaPdot = 0.10;
matrices.maxdeltaEdot = 5/radtodeg;
matrices.maxdeltaAdot = 60/radtodeg;
matrices.maxdeltaRdot = 30/radtodeg;

%% LQR DESIGN

% Simplified matrices: first submodel x1z(z,u,w,theta,q), u1z(deltaE)
Amod1 = [A(1,:);A(2,:);A(4,:);A(6,:);A(9,:)];
A1    = [Amod1(:,1) Amod1(:,2) Amod1(:,4) Amod1(:,6) Amod1(:,9)];
B1    = [B(1,2);B(2,2);B(4,2);B(6,2);B(9,2)];
C1    = eye(5);
D1    = zeros(5,1);
% Simplified matrices: second submodel x1V(V), u1V(deltaP)
A1V   = A(11,11);
B1V   = B(11,1);
C1V   = 1;
D1V   = 0;
% Simplified matrices: third submodel x21(phi,psi), u21(p,r)
Amod21 = [A(5,:);A(7,:)];
A21    = [Amod21(:,5) Amod21(:,7)];
Bmod   = [A(5,:);A(7,:)];
B21    = [Bmod(:,8) Bmod(:,10)];
B21inv = inv(B21);
C21    = eye(2);
D21    = zeros(2,2);
% Simplified matrices: fourth submodel x22(p,r), u22(deltaA,deltaR)
Amod22 = [A(8,:);A(10,:)];
A22    = [Amod22(:,8) Amod22(:,10)];
B22    = [B(8,3:4);B(10,3:4)];
C22    = eye(2);
D22    = zeros(2,2);

% Extended matrices: first submodel x1z(z,u,w,theta,q), u1z(deltaE)
A1a = [A1,zeros(5,1);-[1 0 0 0 0 0]];
B1a = [B1;zeros(1,1)];
C1a = eye(6);
D1a = zeros(6,1);
% Extended matrices: second submodel x1V(V), u1V(deltaP)
A1Va = [A1V,0;-[1 0]];
B1Va = [B1V;0];
C1Va = eye(2);
D1Va = zeros(2,1);
% Extended matrices: third submodel x21(phi,psi), u21(p,r)
A2a1 = [A21,zeros(2,2);-eye(2),zeros(2,2)];
B2a1 = [B21;zeros(2,2)];
C2a1 = eye(4);
D2a1 = zeros(4,2);
% Extended matrices: fourth submodel x22(p,r), u22(deltaA,deltaR)
A2a2 = [A22,zeros(2,2);-eye(2),zeros(2,2)];
B2a2 = [B22;zeros(2,2)];
C2a2 = eye(4);
D2a2 = zeros(4,2);

% Calculation of cost matrices Q, R and N: J = int(xT*Q*x+uT*R*u+2*xT*N*u)
% Matrix Q: contribution to the cost J of state variables
Q1  = diag([1/1000^2 1/50^2 1/0.1^2 1/(15*pi/180)^2 1/(5*pi/180)^2 6*10^10]);
Q1V = diag([1/50^2 10^6]);
Q21 = diag([1/(90*pi/180)^2 1/(90*pi/180)^2 10^13 10^13]);
Q22 = diag([1/(30*pi/180)^2 1/(20*pi/180)^2 10^9 10^10]);

% Matrix R: contribution to the cost J of control variables 
R1    = 100000000000000*1/(15*pi/180)^2;
R1V   = 5000000000*1/0.5^2;
R21   = 50000000000*diag([1/(7.5*pi/180)^2 1/(3.5*pi/180)^2]);
R22   = 45000000*diag([1/(15*pi/180)^2 1/(25*pi/180)^2]);

% LQR optimization for extended matrix K: u = -K*x(t)
G1            = ss(A1a,B1a,C1a,D1a);
[K1,S1,E1]    = lqr(G1,Q1,R1);
G1V           = ss(A1Va,B1Va,C1Va,D1Va);
[K1V,S1V,E1V] = lqr(G1V,Q1V,R1V);
G21            = ss(A2a1,B2a1,C2a1,D2a1);
[K21,S21,E21]    = lqr(G21,Q21,R21);
G22            = ss(A2a2,B2a2,C2a2,D2a2);
[K22,S22,E22]    = lqr(G22,Q22,R22);

% Saving data
matrices.K1    = K1;
matrices.K1V   = K1V;
matrices.K21   = K21;
matrices.K22   = K22;
matrices.B21inv= B21inv;

%% MPC DESIGN

MPC = ss(A,B,C,[D D]);

MPC.InputName = {'deltaP','deltaE','deltaA','deltaR','Vxwind','Vywind',...
    'Vzwind','CDwave'};
MPC.OutputName = {'z','u','v','w','phi','theta','psi','p','q','r','V'};
MPC.StateName = {'z','u','v','w','phi','theta','psi','p','q','r','V'};
MPC.InputGroup.Manipulated = [1,2,3,4];
MPC.InputGroup.Measured = [5,6,7,8];
MPC.OutputGroup.Measured = [1,2,3,4,5,6,7,8,9,10,11];

old_status = mpcverbosity('off');

MPCobj = mpc(MPC,T);

MPCobj.PredictionHorizon = 100;
MPCobj.ControlHorizon = 10;

MPCobj.Model.Plant.InputUnit = {'-','º','º','º','m/s','m/s','m/s','-'};
MPCobj.Model.Plant.OutputUnit = {'m','m/s','m/s','m/s','º','º','º',...
    'º/s','º/s','º/s','m/s'};

MPCobj.ManipulatedVariables(1).Max = matrices.maxdeltaP-v_0trim(23);
MPCobj.ManipulatedVariables(1).Min = matrices.mindeltaP-v_0trim(23);
MPCobj.ManipulatedVariables(1).RateMax = matrices.maxdeltaPdot*T;
MPCobj.ManipulatedVariables(1).RateMin = -matrices.maxdeltaPdot*T;

MPCobj.ManipulatedVariables(2).Max = matrices.maxdeltaE-v_0trim(24);
MPCobj.ManipulatedVariables(2).Min = matrices.mindeltaE-v_0trim(24);
MPCobj.ManipulatedVariables(2).RateMax = 4*matrices.maxdeltaEdot*T;
MPCobj.ManipulatedVariables(2).RateMin = -4*matrices.maxdeltaEdot*T;
MPCobj.ManipulatedVariables(2).ScaleFactor = radtodeg;

MPCobj.ManipulatedVariables(3).Max = matrices.maxdeltaA-v_0trim(25);
MPCobj.ManipulatedVariables(3).Min = matrices.mindeltaA-v_0trim(25);
MPCobj.ManipulatedVariables(3).RateMax = matrices.maxdeltaAdot*T;
MPCobj.ManipulatedVariables(3).RateMin = -matrices.maxdeltaAdot*T;
MPCobj.ManipulatedVariables(3).ScaleFactor = radtodeg;

MPCobj.ManipulatedVariables(4).Max = matrices.maxdeltaR-v_0trim(26);
MPCobj.ManipulatedVariables(4).Min = matrices.mindeltaR-v_0trim(26);
MPCobj.ManipulatedVariables(4).RateMax = matrices.maxdeltaRdot*T;
MPCobj.ManipulatedVariables(4).RateMin = -matrices.maxdeltaRdot*T;
MPCobj.ManipulatedVariables(4).ScaleFactor = radtodeg;

MPCobj.OutputVariables(1).Max = 2*v_0trim(1)-v_0trim(1);
MPCobj.OutputVariables(1).Min = 0-v_0trim(1);
MPCobj.OutputVariables(5).Max = acosd(1/5.5)/radtodeg;
MPCobj.OutputVariables(5).Min = -acosd(1/5.5)/radtodeg;
MPCobj.OutputVariables(6).Max = 10/radtodeg;
MPCobj.OutputVariables(6).Min = -10/radtodeg;
MPCobj.OutputVariables(5).ScaleFactor = radtodeg;
MPCobj.OutputVariables(5).ScaleFactor = radtodeg;
MPCobj.OutputVariables(6).ScaleFactor = radtodeg;
MPCobj.OutputVariables(6).ScaleFactor = radtodeg;
MPCobj.OutputVariables(7).ScaleFactor = radtodeg;
MPCobj.OutputVariables(7).ScaleFactor = radtodeg;
MPCobj.OutputVariables(8).ScaleFactor = radtodeg;
MPCobj.OutputVariables(8).ScaleFactor = radtodeg;
MPCobj.OutputVariables(9).ScaleFactor = radtodeg;
MPCobj.OutputVariables(9).ScaleFactor = radtodeg;
MPCobj.OutputVariables(10).ScaleFactor = radtodeg;
MPCobj.OutputVariables(10).ScaleFactor = radtodeg;

MPCobj.Weights.OutputVariables          = [0.25 0 0 0 1 0 0.5 0 30 0 0.25];
MPCobj.Weights.ManipulatedVariables     = [0 2 0.010 0.125];
MPCobj.Weights.ManipulatedVariablesRate = [0.001 3 0.1 0.1];
MPCobj.Weights.ECR                      = 1000000000;

matrices.MPC = MPCobj;

end