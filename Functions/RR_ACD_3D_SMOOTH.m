function [pathref,Stot,Zm,Vm,psi_initial,OPTIMAL,impact,deviation,C,Z,n] = RR_ACD_3D_SMOOTH(flight_alt0,V0,Vmin,Vmax,ac0,V,Vpsi)

% This function contains two principal algorithms: the first one, a 
% Recursive Rewarding Adaptive Cell Decomposition algorithm (RR-ACD), which 
% creates a dynamic trajectory for the aircraft to follow according to the
% restrictions imposed (ground elevations, danger areas, radar areas and 
% the slope in the bomb dropping); the second one, a simple 3D Smooth
% Path Planning algorithm that tries to find the best trajectory by means 
% of the introduction of radii of gyration at each waypoint. All 
% restrictions, decomposition levels, precision parameters for the 3D 
% Smooth algorithm and minimum load factor values are specified in this 
% function.


%% ORIGIN AND DESTINATION

% Korea Peninsula: Bachelor's Degree Thesis, Álvaro Ortiz
% originlat=sex2dec('N037°31''33.00"');
% originlon=sex2dec('E126°55''19.00"');
% destinlat=sex2dec('N038°24''48.15"');
% destinlon=sex2dec('E127°17''25.77"');

% Alaska: Article from Journal Electronics
originlat = 64.650532;
originlon = -147.066893;
destinlat = 65.830175;
destinlon = -143.407567;

% Altitude from the ground at which the aircraft should drop the bombs 
bomb_alt    = 200;

% Diagonal longitude and its azimuth by the great circle distance
[s,psi0,~] = orto_distazi(originlat,originlon,destinlat,destinlon);

%% DOMAIN DEFINITION

xf = s*cosd(psi0);
yf = s*sind(psi0);
zf = 2*flight_alt0;

%% RESTRICTION DEFINITION

% CIRCULAR AREAS 

% Korea Peninsula: Bachelor's Degree Thesis, Álvaro Ortiz
% circlelat1   = sex2dec('N038°13''13.75"');
% circlelon1   = sex2dec('E127°17''19.86"');
% circlelat2   = sex2dec('N038°09''31.87"');
% circlelon2   = sex2dec('E126°28''55.40"');
% Radii and minimum altitude for restriction
% r    = [18000 60000];
% zmin = [0 700];
% zmax = [zf zf];

% Alaska: Article from Journal Electronics
% Danger area 1
circlelat1 =  65.149993;
circlelon1 = -145.069592;
% Radar area 1
circlelat2 = 66.12070;
circlelon2 = -139.605343;
% Danger area 2
circlelat3 = 65.226888;
circlelon3 = -146.638420;
% Radii and minimum altitude for restriction
r    = [20000 235000 45000];
zmin = [0 900 0];
zmax = [zf zf zf];

% Calculation of centers in Cartesian coordinates, collision objects for
% danger and radar areas and matrices "eps" and "eps2" that will be needed
% by the rewarding function.
[c1,c1psi,~] = orto_distazi(originlat,originlon,circlelat1,circlelon1);
[c2,c2psi,~] = orto_distazi(originlat,originlon,circlelat2,circlelon2);
[c3,c3psi,~] = orto_distazi(originlat,originlon,circlelat3,circlelon3);
c            = [c1 c2 c3];
cpsi         = [c1psi c2psi c3psi];
[cy,xc,yc,eps,eps2] = circle_restriction(c,cpsi,r,zmin,zmax,xf,yf);
Mcy = [xc;yc;r;zmin;zmax]; 

% GROUND ELEVATION

% Generation of collision object that represents the ground elevations and
% calculation of several parameters from the Excel file which contains the
% elevation data. 
[elev,Z,C,n] = orography_restriction(xf,yf);
Zmax = max(Z(:));
nC   = length(C);
nx   = xf/(nC/n-1);
ny   = yf/(n-1);
nv   = [n nx ny nC];

% DROPPING SLOPE

% Ideal dropping altitude
alt_imp = Z(end,end)+bomb_alt;
% Dropping angle (pitch)
% Korea Peninsula: Bachelor's Degree Thesis, Álvaro Ortiz
% a1      = 5
% Alaska: Article from Journal Electronics
a1      = 2;
% Calculation of the collision object
thro    = throwing_restriction(xf,yf,zf,alt_imp,a1);

%% INITIAL SPACE DECOMPOSITION

% Decomposition level for the initial complete space and generation of
% rectangloids, which are initially defined as free ones.
max_ini_divs = 5;
cbi          = initial_space_division(max_ini_divs,xf,yf,zf);
free_i       = [1:size(cbi,3)];

%% TRAJECTORY CALCULATION

% Initial and final points of the trajectory and distances between
% instantaneous location to qf (Mdis) and between qi and qf.
qi = [0 0 flight_alt0];
qf = [xf yf Z(end,end)+bomb_alt];
Mdir = sqrt((qi(1)-qf(1))^2+(qi(2)-qf(2))^2+(qi(3)-qf(3))^2);
Mdis = Mdir;

% Ponderation parameters rho1 and rho2
% Korea Peninsula: Bachelor's Degree Thesis, Álvaro Ortiz
% rho1 = [10000 10 10];
% rho2 = [5000 50 500];
% Alaska: Article from Journal Electronics
rho1 = [4500 10 10];
rho2 = [5000 50 500];

% Local decomposition level, initial rectangloid finding and trajectory
% parameters start.
max_cb_divs = max(6-max_ini_divs,3);
ski = cube_finding(qi,cbi,free_i);
sk  = ski;
sk_ctr = qi;
sf = cube_finding(qf,cbi,free_i);
path  = qi;
sk1 = [];
theta = [0];
psi   = [atan2d(yf,xf)];

% RR-ACD ALGORITHM: calculation of a sub-optimal trajectory. Firstly,
% past-flew rectangloids are removed and proximity to radar area is
% examined, then neighbors are found. Then, Mdis and the allowed centers of
% each rectangloid are obtained, repeating a new decomposition whether no
% possible centers are found. The rewarding function is applied and the
% best center is chosen, afterwards, the "eps2" matrix is defined as empty
% if the aircraft has traversed the bottom part of the radar area perimeter
% or the distance to qf is minimum. 

try
while Mdis > 15000 %For Korea Peninsula: > 3500
    if sk == ski
        free_i(:,find(free_i == sk)) = [];
    else
        for j = 1:length(sk1)
            free_i(:,find(free_i == sk1(j))) = [];
        end
    end
    if r(find(zmin > 0))+30000 > sqrt((path(end,1)-xc(find(zmin > 0)))^2 ...
        +(path(end,2)-yc(find(zmin > 0)))^2)
        radar = 1;
    else
        radar = 0;
    end
    sk1  = neighbourhood(sk,cbi,free_i,radar);
    if length(sk1) == 0
        error;
    end
    Mdis   = sqrt((qf(1)-sk_ctr(1))^2+(qf(2)-sk_ctr(2))^2+(qf(3)-...
        sk_ctr(3))^2);
    maxdiv = max_cb_divs;
    ctrs   = [];
    while length(ctrs) == 0
        for i = sk1
            ctrs_i = free_space_MACD(sk_ctr,cbi(:,:,i),psi(end),maxdiv,...
                cy,elev,thro,Zmax,nv,Mcy);
            %cb(:,:,(size(cb,3)+1):(size(cb,3)+size(cb_i,3))) = cb_i;
            if length(ctrs_i) ~= 0
                ctrs((size(ctrs,1)+1):(size(ctrs,1)+size(ctrs_i,1)),:)...
                    = [ctrs_i i*ones(size(ctrs_i,1),1)];
            end
        end
        if maxdiv > max_cb_divs+1
            break
        end
        maxdiv = maxdiv+1;
    end
    [best,theta(length(theta)+1),psi(length(psi)+1)] = rewards(ctrs,...
        sk_ctr,qf,Mdir,Mdis,rho1,rho2,theta(end),psi(end),eps,eps2);
    path   = [path;best(1:3)];
    sk_ctr = best(1:3);
    sk     = best(4);
    Mdis   = sqrt((qf(1)-sk_ctr(1))^2+(qf(2)-sk_ctr(2))^2+(qf(3)-...
        sk_ctr(3))^2);
    if (Mdis < 15000)||(r(find(zmin > 0))-25000 > sqrt((path(end,1)-...
            xc(find(zmin > 0)))^2+(path(end,2)-yc(find(zmin > 0)))^2))
        eps2 = [];
    end
end

catch
    fprintf('Error in path calculation. Try to change algorithm parameters\n')
end

theta(length(theta)+1) = atand((qf(3)-sk_ctr(3))/sqrt((qf(1)-...
    sk_ctr(1))^2+(qf(2)-sk_ctr(2))^2));
psi(length(psi)+1)     = atan2d((qf(2)-sk_ctr(2)),(qf(1)-sk_ctr(1)));
path = [path;qf];

psi_initial = psi(2)*pi/180;

%% TERRAIN ELEVATION AND PATH PLOT

% Obstacles and trajectory:
figure()
plot3(transpose(path(:,1)),transpose(path(:,2)),transpose(path(:,3)),...
    'r--','linewidth',3.5);
hold on
show(thro);
hold on
for m = 1:length(cy)
    show(cy(m));
    hold on
end
for i = 1:size(path,1)
    scatter3(path(i,1),path(i,2),path(i,3),25,'g','filled',...
        'MarkerEdgeColor','k');
    hold on
end
hold on
terrain_elevation(xf,yf,C,Z,n);
zlim([0 zf]);
set(gca,'DataAspectRatio',[1 1 0.05])
% set(gca,'DataAspectRatio',[0.75*(xf/yf)*7.5 7.5 1000])
view(210,55)
% view(180,90)
% title('Dynamic Path');

% Plot of neighbor rectangloids:
% figure()
% free_space(sk1,cbi,xf,yf,zf,0.85);

% Other types of plots:
% hold on
% free_space(sk1(7:9),cbi,xf,yf,zf,0.85,[1 1 0]);
% hold on
% free_space([sk1(1) sk1(4) sk1(6)],cbi,xf,yf,zf,0.85,[0 0 1]);
% hold on
% free_space([sk1(2) sk1(3) sk1(5) sk1(10) sk1(11)],cbi,xf,yf,zf,...
% 0.85,[0.5 0.5 0.5]);

% hold on;
% for i = 1:size(ctrs,1)
%     scatter3(ctrs(i,1),ctrs(i,2),ctrs(i,3),20,'g','filled',...
% 'MarkerEdgeColor','k');
%     hold on
% end
% hold on
% scatter3(sk_ctr(1),sk_ctr(2),sk_ctr(3),30,'r','filled',...
% 'MarkerEdgeColor','k');

% figure()
% free_space(sk1,cbi,xf,yf,zf,0.85);
% hold on;
% % show(thro);
% % hold on;
% plot3(transpose(path(:,1))./1000,transpose(path(:,2))./1000,...
% transpose(path(:,3)),'r--','linewidth',2.5);
% for i = 1:size(path,1)
%     scatter3(path(i,1)/1000,path(i,2)/1000,path(i,3),20,'g','filled',...
% 'MarkerEdgeColor','k');
%     hold on
% end;
% for i = 1:length(ctrs)
% scatter3(ctrs(i,1)/1000,ctrs(i,2)/1000,ctrs(i,3),50,'b','filled',...
% 'MarkerEdgeColor','k');
% end

% print(gcf,'foo.png','-dpng','-r600');

%% PATH OPTIMIZATION

% Linearization velocity and altitude, accuracy of the soften trajectory 
% and 3D Smooth Path Planning calculation.
Vm   = (Vmin+Vmax)/2;
resolution = 20;
[pathref,Stot] = path_optimization(path,theta,psi,Vmin,Vmax,V0,ac0,...
    resolution,cy,elev,thro,nv,Zmax);
Zm = (max(pathref(:,3))+min(pathref(:,3)))/2;

%% PLOT DYNAMIC PATH

% Soften and RR-ACD trajectories
figure()
plot3(transpose(path(:,1))/1000,transpose(path(:,2))/1000,transpose...
    (path(:,3)),'r','linewidth',1.25);
hold on
scatter3(path(1,1)/1000,path(1,2)/1000,path(1,3),50,'g','filled',...
    'MarkerEdgeColor','k')
hold on
scatter3(path(end,1)/1000,path(end,2)/1000,path(end,3),50,'r','filled',...
    'MarkerEdgeColor','k')
hold on
% scatter3(O(1)/1000,O(2)/1000,O(3),20,'g','filled','MarkerEdgeColor','k')
% hold on
% scatter3(s1(1)/1000,s1(2)/1000,s1(3),20,'g','filled','MarkerEdgeColor','k')
% hold on
% scatter3(s2(1)/1000,s2(2)/1000,s2(3),20,'g','filled','MarkerEdgeColor','k')
% hold on
plot3(transpose(pathref(:,1))/1000,transpose(pathref(:,2))/1000,...
    transpose(pathref(:,3)),'b','linewidth',2);
set(gca,'Ydir','reverse');
xlabel('x [km]','FontSize',10,'FontWeight','bold')
ylabel('y [km]','FontSize',10,'FontWeight','bold')
zlabel('z [m]','FontSize',10,'FontWeight','bold')
xlim([0 xf/1000]);
ylim([0 yf/1000]);
zlim([min(path(:,3)) path(1,3)+10]);
% view(292.5,3.5)
view(270,90)
grid on

% Velocity profile: 
figure()
plot(Stot./1000,transpose(pathref(:,4)),'r','linewidth',1.25);
xlabel('S [km]','FontSize',10,'FontWeight','bold')
ylabel('V(S) [m/s]','FontSize',10,'FontWeight','bold')
xlim([0 Stot(end)/1000])
ylim([165 245])
%title('Reference velocity in function of curvilinear distance')
grid on

% Yaw time variation rate:
figure()
plot(Stot./1000,transpose(pathref(:,6)),'r','linewidth',1.0);
xlabel('S [km]','FontSize',10,'FontWeight','bold')
ylabel('d\psi /dt [º/s]','FontSize',10,'FontWeight','bold')
xlim([0 Stot(end)/1000])
%title('Yaw variation at each point in the trajectory')
grid on

% % Flight path angle:
% figure()
% plot(Stot./1000,transpose(pathref(:,7)),'r','linewidth',1.5);
% xlabel('S [km]','FontSize',12,'FontWeight','bold')
% ylabel('\theta [º]','FontSize',12,'FontWeight','bold')
% xlim([0 Stot(end)/1000])
% grid on

%% DROPPING POINT ESTIMATION

% V and Vdir defined in main file
% Calculation of the most appropriate dropping point over the trajectory.
% prmts will include the flight parameters of the bombs, dropping them from
% each point of the trajectory, while prmts_v contains a matrix with all of
% them. The best option will be the one that minimize the error in impact.
% Finally, the results for the this option are recorded in prmts.

prmts_v = [];
pos_v   = [];
x = linspace(0,xf,length(C)/n);
y = transpose(C(1:n,2));
first_dis = 1500;
for i = size(pathref,1):-1:1
    dist = sqrt((pathref(i,1)-xf)^2+(pathref(i,2)-yf)^2);
    if dist > first_dis
        i_initial = size(pathref,1)-i;
        break;
    end
end
for i = i_initial:-1:0
    prmts = bomb_throwing(pathref(end-i,:),x,y,Z,V,Vpsi);
    pos_v   = [pos_v;[size(prmts_v,1)+1 size(prmts_v,1)+size(prmts,1)]];
    prmts_v = [prmts_v;prmts];
    if (abs(prmts(end,9)) > abs(xf)+50)&&(abs(prmts(end,10)) > abs(yf)+50)
        break;
    end
end
err = [];
for i = 1:size(pos_v,1)
    err = [err sqrt((prmts_v(pos_v(i,2),9)-xf)^2+(prmts_v(pos_v(i,2),10)-yf)^2)];
end
err2   = sort(err);
MINpos = find(round(err,1)==round(err2(1),1));
OPTIMAL = size(pathref,1)-(i_initial-MINpos+1);

% Dropping point selection algorithm plot:
figure()
for i = 1:size(pos_v,1)
    plot3(transpose(prmts_v(pos_v(i,1):pos_v(i,2),9))/1000,...
       transpose(prmts_v(pos_v(i,1):pos_v(i,2),10))/1000,transpose...
       (prmts_v(pos_v(i,1):pos_v(i,2),11)),'r','linewidth',1.0);
    hold on
    scatter3(prmts_v(pos_v(i,1),9)/1000,prmts_v(pos_v(i,1),10)/1000,...
        prmts_v(pos_v(i,1),11),30,'g','filled','MarkerEdgeColor','k')
    hold on
    scatter3(prmts_v(pos_v(i,2),9)/1000,prmts_v(pos_v(i,2),10)/1000,...
        prmts_v(pos_v(i,2),11),20,'r','filled','MarkerEdgeColor','k')
    hold on
end
plot3(transpose(prmts_v(pos_v(MINpos,1):pos_v(MINpos,2),9))/1000,...
       transpose(prmts_v(pos_v(MINpos,1):pos_v(MINpos,2),10))/1000,transpose...
       (prmts_v(pos_v(MINpos,1):pos_v(MINpos,2),11)),'b','linewidth',1.5);
hold on
scatter3(pathref(OPTIMAL,1)/1000,pathref(OPTIMAL,2)/1000,pathref(OPTIMAL,3),120,'b','filled','MarkerEdgeColor','k')
hold on
scatter3(xf/1000,yf/1000,Z(end,end),120,'k','filled','MarkerEdgeColor','k')
hold on
plot3(transpose(pathref(:,1))/1000,transpose(pathref(:,2))/1000,transpose(pathref(:,3)),'k--','linewidth',1.5);
hold on
[X,Y] = meshgrid(x./1000,y./1000);
surf(X,Y,Z);
hold off
set(gca,'Ydir','reverse');
xlabel('x [km]','FontSize',12,'FontWeight','bold')
ylabel('y [km]','FontSize',12,'FontWeight','bold')
zlabel('z [m]','FontSize',12,'FontWeight','bold')
xlim([min(prmts_v(:,9))/1000 max(prmts_v(:,9))/1000]);
ylim([min(prmts_v(:,10))/1000 max(prmts_v(:,10))/1000]);
zlim([-10+min([Z(end,end);prmts_v(:,11)]) max(prmts_v(:,11))]);
%legend('Reference Path','3D Smooth Path');
grid on

prmts = bomb_throwing(pathref(OPTIMAL,:),x,y,Z,V,Vpsi);
impact  = [prmts(end,9) prmts(end,10)];
deviation = sqrt((prmts(end,9)-xf)^2+(prmts(end,10)-yf)^2);

% Flight parameters plot:
figure()
subplot(3,2,1)
plot(transpose(prmts(:,1)),transpose(sqrt(prmts(:,2).^2+prmts(:,3).^2+prmts(:,3).^2)),'b','linewidth',1)
ylabel('V [m/s]','FontSize',10,'FontWeight','bold')
xlim([0 prmts(end,1)])
grid on
subplot(3,2,2)
plot(transpose(prmts(:,1)),transpose(prmts(:,11)),'b','linewidth',1)
ylabel('z [m]','FontSize',10,'FontWeight','bold')
xlim([0 prmts(end,1)])
grid on
subplot(3,2,3)
plot(transpose(prmts(:,1)),transpose(prmts(:,5))*180/pi,'b','linewidth',1)
ylabel('\theta [º]','FontSize',10,'FontWeight','bold')
xlim([0 prmts(end,1)])
grid on
subplot(3,2,4)
plot(transpose(prmts(:,1)),transpose(prmts(:,6))*180/pi,'b','linewidth',1)
ylabel('\psi [º]','FontSize',10,'FontWeight','bold')
xlim([0 prmts(end,1)])
grid on
subplot(3,2,5)
plot(transpose(prmts(:,1)),transpose(prmts(:,7))*180/pi,'b','linewidth',1)
xlabel('Time [s]','FontSize',10,'FontWeight','bold')
ylabel('q [º/s]','FontSize',10,'FontWeight','bold')
xlim([0 prmts(end,1)])
grid on
subplot(3,2,6)
plot(transpose(prmts(:,1)),transpose(prmts(:,8))*180/pi,'b','linewidth',1)
xlabel('Time [s]','FontSize',10,'FontWeight','bold')
ylabel('r [º/s]','FontSize',10,'FontWeight','bold')
xlim([0 prmts(end,1)])
grid on

% Other plots (combination of dropping results utilizing different wind
% conditions):

% figure()
% subplot(3,2,1)
% plot(transpose(prmts1(:,1)),transpose(sqrt(prmts1(:,2).^2+prmts1(:,3).^2+prmts1(:,3).^2)),'b','linewidth',1)
% hold on
% plot(transpose(prmts2(:,1)),transpose(sqrt(prmts2(:,2).^2+prmts2(:,3).^2+prmts2(:,3).^2)),'r','linewidth',1)
% hold on
% plot(transpose(prmts3(:,1)),transpose(sqrt(prmts3(:,2).^2+prmts3(:,3).^2+prmts3(:,3).^2)),'g','linewidth',1)
% hold on
% plot(transpose(prmts4(:,1)),transpose(sqrt(prmts4(:,2).^2+prmts4(:,3).^2+prmts4(:,3).^2)),'k','linewidth',1)
% hold on
% plot(transpose(prmts5(:,1)),transpose(sqrt(prmts5(:,2).^2+prmts5(:,3).^2+prmts5(:,3).^2)),'m','linewidth',1)
% ylabel('V [m/s]','FontSize',10,'FontWeight','bold')
% xlim([0 max([prmts1(end,1) prmts2(end,1) prmts3(end,1) prmts4(end,1) prmts5(end,1)])])
% grid on
% 
% subplot(3,2,2)
% plot(transpose(prmts1(:,1)),transpose(prmts1(:,11)),'b','linewidth',1)
% hold on
% plot(transpose(prmts2(:,1)),transpose(prmts2(:,11)),'r','linewidth',1)
% hold on
% plot(transpose(prmts3(:,1)),transpose(prmts3(:,11)),'g','linewidth',1)
% hold on
% plot(transpose(prmts4(:,1)),transpose(prmts4(:,11)),'k','linewidth',1)
% hold on
% plot(transpose(prmts5(:,1)),transpose(prmts5(:,11)),'m','linewidth',1)
% ylabel('z [m]','FontSize',10,'FontWeight','bold')
% xlim([0 max([prmts1(end,1) prmts2(end,1) prmts3(end,1) prmts4(end,1) prmts5(end,1)])])
% grid on
% 
% subplot(3,2,3)
% plot(transpose(prmts1(:,1)),transpose(prmts1(:,5))*180/pi,'b','linewidth',1)
% hold on
% plot(transpose(prmts2(:,1)),transpose(prmts2(:,5))*180/pi,'r','linewidth',1)
% hold on
% plot(transpose(prmts3(:,1)),transpose(prmts3(:,5))*180/pi,'g','linewidth',1)
% hold on
% plot(transpose(prmts4(:,1)),transpose(prmts4(:,5))*180/pi,'k','linewidth',1)
% hold on
% plot(transpose(prmts5(:,1)),transpose(prmts5(:,5))*180/pi,'m','linewidth',1)
% ylabel('\theta [º]','FontSize',10,'FontWeight','bold')
% xlim([0 max([prmts1(end,1) prmts2(end,1) prmts3(end,1) prmts4(end,1) prmts5(end,1)])])
% grid on
% 
% subplot(3,2,4)
% plot(transpose(prmts1(:,1)),transpose(prmts1(:,6))*180/pi,'b','linewidth',1)
% hold on
% plot(transpose(prmts2(:,1)),transpose(prmts2(:,6))*180/pi,'r','linewidth',1)
% hold on
% plot(transpose(prmts3(:,1)),transpose(prmts3(:,6))*180/pi,'g','linewidth',1)
% hold on
% plot(transpose(prmts4(:,1)),transpose(prmts4(:,6))*180/pi,'k','linewidth',1)
% hold on
% plot(transpose(prmts5(:,1)),transpose(prmts5(:,6))*180/pi,'m','linewidth',1)
% ylabel('\psi [º]','FontSize',10,'FontWeight','bold')
% xlim([0 max([prmts1(end,1) prmts2(end,1) prmts3(end,1) prmts4(end,1) prmts5(end,1)])])
% grid on
% 
% subplot(3,2,5)
% plot(transpose(prmts1(:,1)),transpose(prmts1(:,7))*180/pi,'b','linewidth',1)
% hold on
% plot(transpose(prmts2(:,1)),transpose(prmts2(:,7))*180/pi,'r','linewidth',1)
% hold on
% plot(transpose(prmts3(:,1)),transpose(prmts3(:,7))*180/pi,'g','linewidth',1)
% hold on
% plot(transpose(prmts4(:,1)),transpose(prmts4(:,7))*180/pi,'k','linewidth',1)
% hold on
% plot(transpose(prmts5(:,1)),transpose(prmts5(:,7))*180/pi,'m','linewidth',1)
% xlabel('Time [s]','FontSize',10,'FontWeight','bold')
% ylabel('q [º/s]','FontSize',10,'FontWeight','bold')
% xlim([0 max([prmts1(end,1) prmts2(end,1) prmts3(end,1) prmts4(end,1) prmts5(end,1)])])
% grid on
% 
% subplot(3,2,6)
% plot(transpose(prmts1(:,1)),transpose(prmts1(:,8))*180/pi,'b','linewidth',1)
% hold on
% plot(transpose(prmts2(:,1)),transpose(prmts2(:,8))*180/pi,'r','linewidth',1)
% hold on
% plot(transpose(prmts3(:,1)),transpose(prmts3(:,8))*180/pi,'g','linewidth',1)
% hold on
% plot(transpose(prmts4(:,1)),transpose(prmts4(:,8))*180/pi,'k','linewidth',1)
% hold on
% plot(transpose(prmts5(:,1)),transpose(prmts5(:,8))*180/pi,'m','linewidth',1)
% xlabel('Time [s]','FontSize',10,'FontWeight','bold')
% ylabel('r [º/s]','FontSize',10,'FontWeight','bold')
% xlim([0 max([prmts1(end,1) prmts2(end,1) prmts3(end,1) prmts4(end,1) prmts5(end,1)])])
% grid on

end
