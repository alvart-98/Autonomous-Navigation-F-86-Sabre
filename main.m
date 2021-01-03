%% AUTONOMOUS NAVIGATION FOR AN F-86 SABRE

% Author: Álvaro Ortiz Moya

% Procedure: firstly, the initial conditions for altitude and velocity, 
% maximum and minimum velocity allowed, the acceleration parameter and wind 
% parameters are defined. After, the dynamic trajectory (coordinates of 
% waypoints and velocities that the aircraft must have at each one of them) 
% are calculated by means of the function RR_ACD_3D_SMOOTH.m (all 
% constraint parameters and coordinates for initial and final points are 
% introduced in this function). Then, the linearization process is made and
% the trimmed point is calculated on the basis of linearization conditions 
% of altitude, velocity and yaw angle. Once this has been accomplished, 
% control surfaces and throttle lever limits are introduced and the control 
% law matrices and parameters are defined to allow control by MPC and LQR 
% (everything is saved in matrices.mat struct). Finally, the simulation is 
% performed and different flight parameters are plotted that provide enough
% information to clarify the capabilities of each controller. In addition,
% a FlightGear simulation can be made, showing the aircraft in the real
% motion. 

clc
clear all
close all

addpath('../../Autonomous_navigation_F_86_Sabre/Functions',...
    '../../Autonomous_navigation_F_86_Sabre/Simulation');

%% DYNAMIC TRAJECTORY DESIGN AND INITIAL CONDITIONS

% Initial altitude and velocity, minimum and maximum speeds and
% acceleration rate
alt0 = 1200;
V0   = 200;
Vmin = 190;
Vmax = 240;
acc  = 1/250;

% Wind model
% Mean values
Vwind = 15;
Vdir  = 180;
Velev = 0;
% Maximum and minimum values
Vwindmax  = 18;
Vwindmin  = 12;
Vdirmax   = 195;
Vdirmin   = 165;
Velevmax  = 3;
Velevmin  = -3;
% Standard deviations
VdesvV    = 1;
Vdesvdir  = 5;
Vdesvelev = 1;
% Wind parameters vector
WIND = [Vwind Vdir Velev Vwindmax Vwindmin Vdirmax Vdirmin Velevmax ...
    Velevmin VdesvV Vdesvdir Vdesvelev];

% Dynamic trajectory calculation; total curvilinear distance; linearization
% velocity, altitude and yaw angle; optimal dropping point; impact
% coordinates; deviation in impact; ground elevation parameters
[wp,Stot,Zm,Vm,PSI0,optimal,imp,dev,C,Z,n] = RR_ACD_3D_SMOOTH(alt0,V0,...
    Vmin,Vmax,acc,Vwind,Vdir);
path.wp = wp;
path.S  = Stot;
path.op = optimal;
path.impact = imp;
path.deviation = dev;
path.C  = C;
path.Z  = Z;
path.n  = n;
% Path struct is saved
save('path.mat','path');

%% LINEARIZATION

% Linearization process employing the values of altitude, velocity and yaw
% angle previously defined. All coefficients for control matrices are 
% symbolic, as a function of linearization altitude, velocity and yaw 
% angle. Initial values of each variable in the simulation are also 
% recorded.  
linearization = linearization_f86(Zm,Vm,PSI0,alt0,V0);
% Linearization struct is saved
save('linearization.mat','linearization')

%% STATE-SPACE MATRICES AND COEFFICIENTS

% Sample time 
T       = 0.021;
% Simulation time: this value is only introduced as a reference parameter,
% since the simulation will stop when the end of the reference trajectory
% is attained. 
simtime = 100000*T;

% Creation of control matrices for LQR and MPC and wind model values
matrices = matrices_generator(linearization,simtime,T,WIND);

% Matrices struct is saved. Every time that the matrices_generator function
% is applied, new values for the wind model are given due to the its 
% normalized distribution.
% save('matrices.mat','matrices')
 
%% SIMULATION

s = tf('s');

% Inclusion of initial dynamic trajectory in which the controllers are
% stabilized. This part will be removed in the final study of results.
pathreal = path.wp;
init = [-4000:20:-20]';
path.wp = [[init*[cosd(path.wp(1,5)) sind(path.wp(1,5))]...
    ones(length(init),1)*path.wp(1,3) ones(length(init),1)*path.wp(1,4)...
    ones(length(init),1)*path.wp(1,5) ones(length(init),1)*path.wp(1,6)...
    zeros(length(init),1)];path.wp];

% Simulations start: Linear Quadratic Regulator and Model Predictive
% Control
simout1 = sim('control_f86_LQR');
simout2 = sim('control_f86_MPC');

% Elimination of results regarding the initial dynamic trajectory part for
% stabilization of controllers
simout1 = results(simout1);
simout2 = results(simout2);
path.wp = pathreal;

% plot_maker(simout2,path.wp)
% Save simulation results
% save('LQR_Alaska.mat','simout1')
% save('MPC_Alaska.mat','simout2')

%% ANALYSIS OF BOMBS IMPACTS

% Deviations in impact with respect to the objective (dev), indexes (for 
% simulations results) of the arrays in which the dropping is more adequate
% (OPT), flight parameters of the bombs (prmts), deviations for dropping
% points with respect to the reference ideal release coordinates (DEV). 
path_1 = [simout1.x simout1.y simout1.z simout1.u simout1.v simout1.w ...
         simout1.theta simout1.psi simout1.q simout1.r];
path_2 = [simout2.x simout2.y simout2.z simout2.u simout2.v simout2.w ...
         simout2.theta simout2.psi simout2.q simout2.r];
[dev1,OPT1,prmts1] = impact_point(path.C,path.Z,path.n,path.wp(end,1),...
    path.wp(end,2),path_1,matrices.wind,matrices.T,'b--');
DEV1 = sqrt((simout1.x(OPT1)-path.wp(path.op,1))^2+(simout1.y(OPT1)-...
    path.wp(path.op,2))^2);
[dev2,OPT2,prmts2] = impact_point(path.C,path.Z,path.n,path.wp(end,1),...
    path.wp(end,2),path_2,matrices.wind,matrices.T,'r--');
DEV2 = sqrt((simout2.x(OPT2)-path.wp(path.op,1))^2+(simout2.y(OPT2)-...
    path.wp(path.op,2))^2);

%% PLOTS

% Blue color assigned to LQR and red one to MPC

% Plot 1: Velocity and altitude analysis
% Plot 2: Main flight angles for each controller case, such as angle of 
% attack, sideslip angle and Euler angles.
% Plot 3: Main flight angles and angular body velocities for LQR
% Plot 4: Main flight angles and angular body velocities for MPC
% Plot 5: Control signals for both controllers 
% Plot 6: Control signals for LQR, y-axis with range defined by the 
% maximum and minimum values for control actions imposed
% Plot 7: Control signals for MPC, y-axis with range defined by the 
% maximum and minimum values for control actions imposed
% Plot 8: Trajectories of the reference case and simulations for each
% controller
% Plot 9: Wind velocities
% Plot 10: Fuel mass 
% Plot 11: Flight parameters of the bombs at each case

% Thickness of lines for parameters
L = 1;
% Thickness of reference lines
Lr = 0.75;

% Plot of the 11 figures
plot_maker_total(simout1,simout2,path.wp,prmts1,prmts2,Lr,L)
% Convert Matlab figure into '.png' file
% print(gcf,'foo.png','-dpng','-r450');

%% FLIGHTGEAR

% Selection of simulation results for one of the controllers
simout = simout1;
% Sample time
dt     = simout.t(2)-simout.t(1);
% Simulation time
simt   = simout.t(end);

% Korea Peninsula: Bachelor's Degree Thesis, Álvaro Ortiz
% originlat=sex2dec('N037°31''33.00"');
% originlon=sex2dec('E126°55''19.00"');

% Alaska: Article from Journal Electronics
originlat = 64.650532;
originlon = -147.066893;

% Convert Euler angles into radians
simout.phi   = simout.phi/matrices.radtodeg;
simout.theta = simout.theta/matrices.radtodeg;
simout.psi   = simout.psi/matrices.radtodeg;

% Start of the simulation
sim('flightgear_simulator');

