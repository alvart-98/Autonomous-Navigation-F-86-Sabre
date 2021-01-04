function [] = plot_maker_total(simout1,simout2,wp,prmts1,prmts2,Lr,L)

% Representation of the results from simulation in Simulink 

%% PLOT 1
figure()
subplot(4,2,1)
plot(transpose(simout1.t),transpose(simout1.Vref),'g--','linewidth',Lr);
hold on
plot(transpose(simout1.t),transpose(simout1.V),'b','linewidth',L);
ylabel('V(t),V_{ref}(t) [m/s]','FontSize',8.5,'FontWeight','bold')
grid on
xlim([0 max(simout1.t(end),simout2.t(end))])
ylim([165 245])
subplot(4,2,2)
plot(transpose(simout2.t),transpose(simout2.Vref),'g--','linewidth',Lr);
hold on
plot(transpose(simout2.t),transpose(simout2.V),'r','linewidth',L);
ylabel('V(t),V_{ref}(t) [m/s]','FontSize',8.5,'FontWeight','bold')
grid on
xlim([0 max(simout1.t(end),simout2.t(end))])
ylim([165 245])
subplot(4,2,3)
plot(transpose(simout1.t),transpose(simout1.Vref)-transpose(simout1.V),...
    'b','linewidth',L);
ylabel('V_{ref}-V(t) [m/s]','FontSize',8.5,'FontWeight','bold')
grid on
xlim([0 max(simout1.t(end),simout2.t(end))])
ylim([-8 6])
subplot(4,2,4)
plot(transpose(simout2.t),transpose(simout2.Vref)-transpose(simout2.V),...
    'r','linewidth',L);
ylabel('V_{ref}(t)-V(t) [m/s]','FontSize',8.5,'FontWeight','bold')
grid on
xlim([0 max(simout1.t(end),simout2.t(end))])
ylim([-8 6])
subplot(4,2,5)
plot(transpose(simout1.t),transpose(simout1.zref),'g--','linewidth',Lr);
hold on
plot(transpose(simout1.t),transpose(simout1.z),'b','linewidth',L);
ylabel('z(t),z_{ref}(t) [m]','FontSize',8.5,'FontWeight','bold')
grid on
xlim([0 max(simout1.t(end),simout2.t(end))])
ylim([470 1230])
subplot(4,2,6)
plot(transpose(simout2.t),transpose(simout2.zref),'g--','linewidth',Lr);
hold on
plot(transpose(simout2.t),transpose(simout2.z),'r','linewidth',L);
ylabel('z(t),z_{ref}(t) [m]','FontSize',8.5,'FontWeight','bold')
grid on
xlim([0 max(simout1.t(end),simout2.t(end))])
ylim([470 1230])
subplot(4,2,7)
plot(transpose(simout1.t),transpose(simout1.zref)-transpose(simout1.z),...
    'b','linewidth',L);
ylabel('z_{ref}(t)-z(t) [m]','FontSize',8.5,'FontWeight','bold')
grid on
xlim([0 max(simout1.t(end),simout2.t(end))])
ylim([-5 5])
xlabel('Time [s]','FontSize',10,'FontWeight','bold')
subplot(4,2,8)
plot(transpose(simout2.t),transpose(simout2.zref)-transpose(simout2.z),...
    'r','linewidth',L);
xlabel('Time [s]','FontSize',10,'FontWeight','bold')
ylabel('z_{ref}(t)-z(t) [m]','FontSize',8.5,'FontWeight','bold')
grid on
xlim([0 max(simout1.t(end),simout2.t(end))])
ylim([-5 5])

%% ANGLE OF ATTACK AND SIDESLIP ANGLE

alpha1 = [];
beta1  = [];
for i = 1:length(simout1.t)
girophi   = [1,0,0;0,cosd(simout1.phi(i)),sind(simout1.phi(i));0,...
    (-1).*sind(simout1.phi(i)),cosd(simout1.phi(i))];
girotheta = [cosd(simout1.theta(i)),0,(-1).*sind(simout1.theta(i));0,...
    1,0;sind(simout1.theta(i)),0,cosd(simout1.theta(i))];
giropsi   = [cosd(simout1.psi(i)),sind(simout1.psi(i)),0;(-1).*...
    sind(simout1.psi(i)),cosd(simout1.psi(i)),0;0,0,1];
MBH       = girophi*girotheta*giropsi; % Local horizon axes to body axes
uvwwind   = MBH*[simout1.Vwinds(i,1);simout1.Vwinds(i,2);...
    simout1.Vwinds(i,3)];
uwind    = uvwwind(1,1);
vwind    = uvwwind(2,1);
wwind    = uvwwind(3,1);
uvw1(1:3,i) = uvwwind;
urel     = simout1.u(i)-uwind;
vrel     = simout1.v(i)-vwind;
wrel     = simout1.w(i)-wwind;
V_total  = sqrt(urel^2+vrel^2+wrel^2);
alpha1    = [alpha1 atand(wrel/urel)];
beta1     = [beta1 asind(vrel/V_total)];
end

alpha2 = [];
beta2  = [];
for i = 1:length(simout2.t)
girophi   = [1,0,0;0,cosd(simout2.phi(i)),sind(simout2.phi(i));0,...
    (-1).*sind(simout2.phi(i)),cosd(simout2.phi(i))];
girotheta = [cosd(simout2.theta(i)),0,(-1).*sind(simout2.theta(i));0,...
    1,0;sind(simout2.theta(i)),0,cosd(simout2.theta(i))];
giropsi   = [cosd(simout2.psi(i)),sind(simout2.psi(i)),0;...
    (-1).*sind(simout2.psi(i)),cosd(simout2.psi(i)),0;0,0,1];
MBH       = girophi*girotheta*giropsi; % Local horizon axes to body axes
uvwwind   = MBH*[simout2.Vwinds(i,1);simout2.Vwinds(i,2);...
    simout2.Vwinds(i,3)];
uwind    = uvwwind(1,1);
vwind    = uvwwind(2,1);
wwind    = uvwwind(3,1);
uvw2(1:3,i) = uvwwind;
urel     = simout2.u(i)-uwind;
vrel     = simout2.v(i)-vwind;
wrel     = simout2.w(i)-wwind;
V_total  = sqrt(urel^2+vrel^2+wrel^2);
alpha2    = [alpha2 atand(wrel/urel)];
beta2     = [beta2 asind(vrel/V_total)];
end

%% PLOT 2

figure()
subplot(5,2,1)
plot(transpose(simout1.t),alpha1,'b','linewidth',L);
ylabel('\alpha(t) [º]','FontSize',10,'FontWeight','bold')
grid on
xlim([0 max(simout1.t(end),simout2.t(end))])
ylim([-1.5 4])
subplot(5,2,2)
plot(transpose(simout2.t),alpha2,'r','linewidth',L);
ylabel('\alpha(t) [º]','FontSize',10,'FontWeight','bold')
grid on
xlim([0 max(simout1.t(end),simout2.t(end))])
ylim([-1.5 4])
subplot(5,2,3)
plot(transpose(simout1.t),beta1,'b','linewidth',L);
ylabel('\beta(t) [º]','FontSize',10,'FontWeight','bold')
grid on
xlim([0 max(simout1.t(end),simout2.t(end))])
ylim([-6 6])
subplot(5,2,4)
plot(transpose(simout2.t),beta2,'r','linewidth',L);
ylabel('\beta(t) [º]','FontSize',10,'FontWeight','bold')
grid on
xlim([0 max(simout1.t(end),simout2.t(end))])
ylim([-6 6])
subplot(5,2,5)
plot(transpose(simout1.t),transpose(simout1.phi),'b','linewidth',L);
ylabel('\phi(t) [º]','FontSize',10,'FontWeight','bold')
grid on
xlim([0 max(simout1.t(end),simout2.t(end))])
ylim([-51 30])
subplot(5,2,6)
plot(transpose(simout2.t),transpose(simout2.phi),'r','linewidth',L);
ylabel('\phi(t) [º]','FontSize',10,'FontWeight','bold')
grid on
xlim([0 max(simout1.t(end),simout2.t(end))])
ylim([-51 30])
subplot(5,2,7)
plot(transpose(simout1.t),transpose(simout1.theta),'b','linewidth',L);
ylabel('\theta(t) [º]','FontSize',10,'FontWeight','bold')
grid on
xlim([0 max(simout1.t(end),simout2.t(end))])
ylim([-1 6])
subplot(5,2,8)
plot(transpose(simout2.t),transpose(simout2.theta),'r','linewidth',L);
ylabel('\theta(t) [º]','FontSize',10,'FontWeight','bold')
grid on
xlim([0 max(simout1.t(end),simout2.t(end))])
ylim([-1 6])
subplot(5,2,9)
plot(transpose(simout1.t),transpose(simout1.psi),'b','linewidth',L);
xlabel('Time [s]','FontSize',10,'FontWeight','bold')
ylabel('\psi(t) [º]','FontSize',10,'FontWeight','bold')
grid on
xlim([0 max(simout1.t(end),simout2.t(end))])
ylim([-6 76])
subplot(5,2,10)
plot(transpose(simout2.t),transpose(simout2.psi),'r','linewidth',L);
xlabel('Time [s]','FontSize',10,'FontWeight','bold')
ylabel('\psi(t) [º]','FontSize',10,'FontWeight','bold')
grid on
xlim([0 max(simout1.t(end),simout2.t(end))])
ylim([-6 76])

%% PLOT 3

figure()
subplot(4,2,1)
plot(transpose(simout1.t),alpha1,'b','linewidth',L);
ylabel('\alpha(t) [º]','FontSize',10,'FontWeight','bold')
grid on
subplot(4,2,2)
plot(transpose(simout1.t),beta1,'b','linewidth',L);
ylabel('\beta(t) [º]','FontSize',10,'FontWeight','bold')
grid on
subplot(4,2,3)
plot(transpose(simout1.t),transpose(simout1.phi),'b','linewidth',L);
ylabel('\phi(t) [º]','FontSize',10,'FontWeight','bold')
grid on
subplot(4,2,4)
plot(transpose(simout1.t),transpose(simout1.p),'b','linewidth',L);
ylabel('p(t) [º/s]','FontSize',10,'FontWeight','bold')
grid on
subplot(4,2,5)
plot(transpose(simout1.t),transpose(simout1.theta),'b','linewidth',L);
ylabel('\theta(t) [º]','FontSize',10,'FontWeight','bold')
grid on
subplot(4,2,6)
plot(transpose(simout1.t),transpose(simout1.q),'b','linewidth',L);
ylabel('q(t) [º/s]','FontSize',10,'FontWeight','bold')
grid on
subplot(4,2,7)
plot(transpose(simout1.t),transpose(simout1.psi),'b','linewidth',L);
xlabel('Time [s]','FontSize',10,'FontWeight','bold')
ylabel('\psi(t) [º]','FontSize',10,'FontWeight','bold')
grid on
subplot(4,2,8)
plot(transpose(simout1.t),transpose(simout1.r),'b','linewidth',L);
xlabel('Time [s]','FontSize',10,'FontWeight','bold')
ylabel('r(t) [º/s]','FontSize',10,'FontWeight','bold')
grid on

%% PLOT 4

figure()
subplot(4,2,1)
plot(transpose(simout2.t),alpha2,'r','linewidth',L);
ylabel('\alpha(t) [º]','FontSize',10,'FontWeight','bold')
grid on
subplot(4,2,2)
plot(transpose(simout2.t),beta2,'r','linewidth',L);
ylabel('\beta(t) [º]','FontSize',10,'FontWeight','bold')
grid on
subplot(4,2,3)
plot(transpose(simout2.t),transpose(simout2.phi),'r','linewidth',L);
ylabel('\phi(t) [º]','FontSize',10,'FontWeight','bold')
grid on
subplot(4,2,4)
plot(transpose(simout2.t),transpose(simout2.p),'r','linewidth',L);
ylabel('p(t) [º/s]','FontSize',10,'FontWeight','bold')
grid on
subplot(4,2,5)
plot(transpose(simout2.t),transpose(simout2.theta),'r','linewidth',L);
ylabel('\theta(t) [º]','FontSize',10,'FontWeight','bold')
grid on
subplot(4,2,6)
plot(transpose(simout2.t),transpose(simout2.q),'r','linewidth',L);
ylabel('q(t) [º/s]','FontSize',10,'FontWeight','bold')
grid on
subplot(4,2,7)
plot(transpose(simout2.t),transpose(simout2.psi),'r','linewidth',L);
xlabel('Time [s]','FontSize',10,'FontWeight','bold')
ylabel('\psi(t) [º]','FontSize',10,'FontWeight','bold')
grid on
subplot(4,2,8)
plot(transpose(simout2.t),transpose(simout2.r),'r','linewidth',L);
xlabel('Time [s]','FontSize',10,'FontWeight','bold')
ylabel('r(t) [º/s]','FontSize',10,'FontWeight','bold')
grid on

%% PLOT 5

figure()
subplot(4,2,1)
plot(transpose(simout1.t),transpose(simout1.deltaP),'b','linewidth',L);
ylabel('\delta_{P}(t) [-]','FontSize',10,'FontWeight','bold')
xlim([0 max(simout1.t(end),simout2.t(end))])
ylim([0 0.75]);
grid on
subplot(4,2,2)
plot(transpose(simout2.t),transpose(simout2.deltaP),'r','linewidth',L);
ylabel('\delta_{P}(t) [-]','FontSize',10,'FontWeight','bold')
xlim([0 max(simout1.t(end),simout2.t(end))])
ylim([0 0.75]);
grid on
subplot(4,2,3)
plot(transpose(simout1.t),transpose(simout1.deltaA),'b','linewidth',L);
ylabel('\delta_{A}(t) [º]','FontSize',10,'FontWeight','bold')
xlim([0 max(simout1.t(end),simout2.t(end))])
ylim([-5 5]);
grid on
subplot(4,2,4)
plot(transpose(simout2.t),transpose(simout2.deltaA),'r','linewidth',L);
ylabel('\delta_{A}(t) [º]','FontSize',10,'FontWeight','bold')
xlim([0 max(simout1.t(end),simout2.t(end))])
ylim([-5 5]);
grid on
subplot(4,2,5)
plot(transpose(simout1.t),transpose(simout1.deltaE),'b','linewidth',L);
ylabel('\delta_{E}(t) [º]','FontSize',10,'FontWeight','bold')
xlim([0 max(simout1.t(end),simout2.t(end))])
ylim([-3 1]);
grid on
subplot(4,2,6)
plot(transpose(simout2.t),transpose(simout2.deltaE),'r','linewidth',L);
ylabel('\delta_{E}(t) [º]','FontSize',10,'FontWeight','bold')
xlim([0 max(simout1.t(end),simout2.t(end))])
ylim([-3 1]);
grid on
subplot(4,2,7)
plot(transpose(simout1.t),transpose(simout1.deltaR),'b','linewidth',L);
ylabel('\delta_{R}(t) [º]','FontSize',10,'FontWeight','bold')
xlim([0 max(simout1.t(end),simout2.t(end))])
ylim([-28 28]);
grid on
xlabel('Time [s]','FontSize',10,'FontWeight','bold')
subplot(4,2,8)
plot(transpose(simout2.t),transpose(simout2.deltaR),'r','linewidth',L);
xlabel('Time [s]','FontSize',10,'FontWeight','bold')
ylabel('\delta_{R}(t) [º]','FontSize',10,'FontWeight','bold')
xlim([0 max(simout1.t(end),simout2.t(end))])
ylim([-28 28]);
grid on

%% PLOT 6

figure()
subplot(2,2,1)
plot(transpose(simout1.t),transpose(simout1.deltaP),'b','linewidth',L);
ylabel('\delta_{P}(t) [-]','FontSize',10,'FontWeight','bold')
ylim([0 1]);
grid on
subplot(2,2,2)
plot(transpose(simout1.t),transpose(simout1.deltaE),'b','linewidth',L);
ylabel('\delta_{E}(t) [º]','FontSize',10,'FontWeight','bold')
ylim([-22 3]);
grid on
subplot(2,2,3)
plot(transpose(simout1.t),transpose(simout1.deltaA),'b','linewidth',L);
xlabel('Time [s]','FontSize',10,'FontWeight','bold')
ylabel('\delta_{A}(t) [º]','FontSize',10,'FontWeight','bold')
ylim([-16 16]);
grid on
subplot(2,2,4)
plot(transpose(simout1.t),transpose(simout1.deltaR),'b','linewidth',L);
xlabel('Time [s]','FontSize',10,'FontWeight','bold')
ylabel('\delta_{R}(t) [º]','FontSize',10,'FontWeight','bold')
ylim([-26 26]);
grid on

%% PLOT 7

figure()
subplot(2,2,1)
plot(transpose(simout2.t),transpose(simout2.deltaP),'r','linewidth',L);
ylabel('\delta_{P}(t) [-]','FontSize',10,'FontWeight','bold')
ylim([0 1]);
grid on
subplot(2,2,2)
plot(transpose(simout2.t),transpose(simout2.deltaE),'r','linewidth',L);
ylabel('\delta_{E}(t) [º]','FontSize',10,'FontWeight','bold')
ylim([-22 3]);
grid on
subplot(2,2,3)
plot(transpose(simout2.t),transpose(simout2.deltaA),'r','linewidth',L);
xlabel('Time [s]','FontSize',10,'FontWeight','bold')
ylabel('\delta_{A}(t) [º]','FontSize',10,'FontWeight','bold')
ylim([-16 16]);
grid on
subplot(2,2,4)
plot(transpose(simout2.t),transpose(simout2.deltaR),'r','linewidth',L);
xlabel('Time [s]','FontSize',10,'FontWeight','bold')
ylabel('\delta_{R}(t) [º]','FontSize',10,'FontWeight','bold')
ylim([-26 26]);
grid on

%% PLOT 8

figure()
plot3(transpose(wp(:,1))./1000,transpose(wp(:,2))./1000,...
    transpose(wp(:,3)),'g--','linewidth',Lr+0.35)
hold on
plot3(transpose(simout1.x)./1000,transpose(simout1.y)./1000,...
    transpose(simout1.z),'b','linewidth',L+0.75)
hold on
plot3(transpose(simout2.x)./1000,transpose(simout2.y)./1000,...
    transpose(simout2.z),'r','linewidth',L+0.75)
xlabel('x [km]','FontSize',10,'FontWeight','bold')
ylabel('y [km]','FontSize',10,'FontWeight','bold')
zlabel('z [m]','FontSize',10,'FontWeight','bold')
xlim([0 max([wp(end,1) simout1.x(end) simout2.x(end)])]./1000)
ylim([0 max([wp(end,2) simout1.y(end) simout2.y(end)])]./1000)
zlim([min(wp(:,3)) max(wp(:,3))+10])
set(gca,'Ydir','reverse');
view(292.5,8)
% view(270,90)
grid on

%% PLOT 9

figure()
subplot(3,1,1)
plot(transpose(simout1.t),transpose(simout1.Vwinds(:,1)),'b','linewidth',L);
ylabel('V_{w,x}(t) [m/s]','FontSize',10,'FontWeight','bold')
ylim([-20 -10]);
xlim([0 1020]);
grid on
subplot(3,1,2)
plot(transpose(simout1.t),transpose(simout1.Vwinds(:,2)),'b','linewidth',L);
ylabel('V_{w,y}(t) [m/s]','FontSize',10,'FontWeight','bold')
ylim([-8 8]);
xlim([0 1020]);
grid on
subplot(3,1,3)
plot(transpose(simout1.t),transpose(simout1.Vwinds(:,3)),'b','linewidth',L);
xlabel('Time [s]','FontSize',10,'FontWeight','bold')
ylabel('V_{w,z}(t) [m/s]','FontSize',10,'FontWeight','bold')
ylim([-3 3]);
xlim([0 1020]);
grid on

%% PLOT 10

if simout1.Fuel(1) == simout1.Fuel(end)
    min_y = 0;
else
    min_y = min(simout1.Fuel(end),simout2.Fuel(end));
end

figure()
plot(transpose(simout1.t),transpose(simout1.Fuel),'b','linewidth',L);
hold on
plot(transpose(simout2.t),transpose(simout2.Fuel),'r','linewidth',L);
xlabel('Time [s]','FontSize',10,'FontWeight','bold')
ylabel('m_{Fuel}(t) [kg]','FontSize',10,'FontWeight','bold')
xlim([0 max(simout1.t(end),simout2.t(end))])
ylim([min_y simout1.Fuel(1)]);
grid on

%% PLOT 11

figure()
subplot(3,2,1)
plot(transpose(prmts1(:,1)),transpose(sqrt(prmts1(:,2).^2+...
    prmts1(:,3).^2+prmts1(:,3).^2)),'b','linewidth',1)
hold on
plot(transpose(prmts2(:,1)),transpose(sqrt(prmts2(:,2).^2+...
    prmts2(:,3).^2+prmts2(:,3).^2)),'r','linewidth',1)
ylabel('V [m/s]','FontSize',10,'FontWeight','bold')
xlim([0 max(prmts1(end,1),prmts2(end,1))])
grid on
subplot(3,2,2)
plot(transpose(prmts1(:,1)),transpose(prmts1(:,11)),'b','linewidth',L)
hold on
plot(transpose(prmts2(:,1)),transpose(prmts2(:,11)),'r','linewidth',L)
ylabel('z [m]','FontSize',10,'FontWeight','bold')
xlim([0 max(prmts1(end,1),prmts2(end,1))])
grid on
subplot(3,2,3)
plot(transpose(prmts1(:,1)),transpose(prmts1(:,5))*180/pi,'b','linewidth',L)
hold on
plot(transpose(prmts2(:,1)),transpose(prmts2(:,5))*180/pi,'r','linewidth',L)
ylabel('\theta [º]','FontSize',10,'FontWeight','bold')
xlim([0 max(prmts1(end,1),prmts2(end,1))])
grid on
subplot(3,2,4)
plot(transpose(prmts1(:,1)),transpose(prmts1(:,6))*180/pi,'b','linewidth',1)
hold on
plot(transpose(prmts2(:,1)),transpose(prmts2(:,6))*180/pi,'r','linewidth',1)
ylabel('\psi [º]','FontSize',10,'FontWeight','bold')
xlim([0 max(prmts1(end,1),prmts2(end,1))])
grid on
subplot(3,2,5)
plot(transpose(prmts1(:,1)),transpose(prmts1(:,7))*180/pi,'b','linewidth',L)
hold on
plot(transpose(prmts2(:,1)),transpose(prmts2(:,7))*180/pi,'r','linewidth',L)
xlabel('Time [s]','FontSize',10,'FontWeight','bold')
ylabel('q [º/s]','FontSize',10,'FontWeight','bold')
xlim([0 max(prmts1(end,1),prmts2(end,1))])
grid on
subplot(3,2,6)
plot(transpose(prmts1(:,1)),transpose(prmts1(:,8))*180/pi,'b','linewidth',L)
hold on
plot(transpose(prmts2(:,1)),transpose(prmts2(:,8))*180/pi,'r','linewidth',L)
xlabel('Time [s]','FontSize',10,'FontWeight','bold')
ylabel('r [º/s]','FontSize',10,'FontWeight','bold')
xlim([0 max(prmts1(end,1),prmts2(end,1))])
grid on

end