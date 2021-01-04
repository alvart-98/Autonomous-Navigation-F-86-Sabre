function [] = plot_maker(simout,wp)

% Representation of the results from simulation in Simulink 
% First figure: temporary values of the magnitudes which define the
% behaviour of the aircraft, employing body axes
% Second figure: reference and real trajectory of the aircraft, employing
% cartesian axes
% Third figure: Control actions that are needed to carry out the reference
% trajectory implemented in Simulink

figure()
subplot(3,3,1)
plot(transpose(simout.t),transpose(simout.u),'r','linewidth',1.5);
hold on
plot(transpose(simout.t),transpose(simout.V),'b','linewidth',1.5);
xlabel('Time [s]')
ylabel('u(t), V(t) [m/s]')
title('Velocity u in body axes and Total Velocity')
legend('u(t)','V(t)')
grid on
subplot(3,3,2)
plot(transpose(simout.t),transpose(simout.v),'r','linewidth',1.5);
xlabel('Time [s]')
ylabel('v(t) [m/s]')
title('Velocity v in body axes')
grid on
subplot(3,3,3)
plot(transpose(simout.t),transpose(simout.w),'r','linewidth',1.5);
xlabel('Time [s]')
ylabel('w(t) [m/s]')
title('Velocity w in body axes')
grid on
subplot(3,3,4)
plot(transpose(simout.t),transpose(simout.phi),'b','linewidth',1.5);
hold on
plot(transpose(simout.t),transpose(simout.phiref),'r--','linewidth',1);
xlabel('Time [s]')
ylabel('\phi (t) [º]')
title('Euler angle \phi')
legend('\phi (t)','\phi_{ref} (t)')
grid on
subplot(3,3,5)
plot(transpose(simout.t),transpose(simout.theta),'b','linewidth',1.5);
xlabel('Time [s]')
ylabel('\theta (t) [º]')
title('Euler angle \theta')
grid on
subplot(3,3,6)
plot(transpose(simout.t),transpose(simout.psi),'b','linewidth',1.5);
hold on
plot(transpose(simout.t),transpose(simout.rumbref),'r--','linewidth',1);
hold on
plot(transpose(simout.t),transpose(simout.rumb),'r','linewidth',1.5);
xlabel('Time [s]')
ylabel('\psi (t) [º]')
title('Euler angle \psi')
legend('\psi (t)','rumb_{ref} (t)','rumb (t)')
grid on
subplot(3,3,7)
plot(transpose(simout.t),transpose(simout.p),'g','linewidth',1.5);
xlabel('Time [s]')
ylabel('p(t) [º/s]')
title('Angular velocity p in body axes')
grid on
subplot(3,3,8)
plot(transpose(simout.t),transpose(simout.q),'g','linewidth',1.5);
xlabel('Time [s]')
ylabel('q(t) [º/s]')
title('Angular velocity q in body axes')
grid on
subplot(3,3,9)
plot(transpose(simout.t),transpose(simout.r),'g','linewidth',1.5);
xlabel('Time [s]')
ylabel('r(t) [º/s]')
title('Angular velocity r in body axes')
grid on

% Limits in 3D plot
ymaxabs = max(abs(min(simout.y)),max(simout.y));
if ymaxabs > 50
    ylims = [min([min(simout.y),min(wp(:,2))]) ...
            max([max(simout.y),max(wp(:,2))])];
    else ylims = [-50 50];
end

figure()
plot3(transpose(wp(:,1)),transpose(wp(:,2)),transpose(wp(:,3)),'r','linewidth',1.25)
hold on
plot3(transpose(simout.x),transpose(simout.y),transpose(simout.z),'b','linewidth',1.5)
hold on
scatter3(simout.x(1),simout.y(1),simout.z(1),50,[0.5 0 0],'filled','MarkerEdgeColor','k')
hold on
scatter3(simout.x(end),simout.y(end),simout.z(end),50,'g','filled','MarkerEdgeColor','k')
xlabel('x [m]')
ylabel('y [m]')
zlabel('z [m]')
xlim([min([min(simout.x),min(wp(:,1))]) ...
    max([max(simout.x),max(wp(:,1))])])
ylim([ylims]);
zlim([0 max([max(simout.z),max(wp(:,3))])]);
set(gca,'Ydir','reverse');
title('Trajectory from the linearized and reference models ')
legend('Reference','Non-linearized model','Initial point','Final point');
grid on

figure()
subplot(2,2,1)
plot(transpose(simout.t),transpose(simout.deltaP),'b','linewidth',1.5);
xlabel('Time [s]')
ylabel('\delta_{P} (t) [-]')
ylim([0 1]);
title('Thrust lever')
grid on
subplot(2,2,2)
plot(transpose(simout.t),transpose(simout.deltaE),'b','linewidth',1.5);
xlabel('Time [s]')
ylabel('\delta_{E} (t) [º]')
ylim([-30 15]);
title('Elevator deflection')
grid on
subplot(2,2,3)
plot(transpose(simout.t),transpose(simout.deltaA),'r','linewidth',1.5);
xlabel('Time [s]')
ylabel('\delta_{A} (t) [º]')
ylim([-30 30]);
title('Aileron deflection')
grid on
subplot(2,2,4)
plot(transpose(simout.t),transpose(simout.deltaR),'r','linewidth',1.5);
xlabel('Time [s]')
ylabel('\delta_{R} (t) [º]')
ylim([-30 30]);
title('Rudder deflection')
grid on

% figure()
% subplot(3,1,1)
% plot(transpose(simout.t),transpose(simout.Vwinds(:,1)),'b','linewidth',1.5);
% ylabel('wind_{x} (t) [m/s]')
% ylim([-5 30]);
% title('V_{wind}')
% grid on
% subplot(3,1,2)
% plot(transpose(simout.t),transpose(simout.Vwinds(:,2)),'b','linewidth',1.5);
% ylabel('wind_{y} (t) [m/s]')
% ylim([-5 30]);
% grid on
% subplot(3,1,3)
% plot(transpose(simout.t),transpose(simout.Vwinds(:,3)),'b','linewidth',1.5);
% xlabel('Time [s]')
% ylabel('wind_{y} (t) [m/s]')
% ylim([-5 5]);
% grid on

% Deviations
% xdev = abs(simout.x(end)-simout.xref(end));
% ydev = abs(simout.y(end)-simout.yref(end));
% zdev = abs(simout.z(end)-simout.zref(end));
% fprintf('Absolute deviation in x position is: %g m \n',xdev)
% fprintf('Absolute deviation in y position is: %g m \n',ydev)
% fprintf('Absolute deviation in z position is: %g m \n',zdev)

end