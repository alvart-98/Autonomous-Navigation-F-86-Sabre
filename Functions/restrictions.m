function [] = restrictions(cy,elev,xf,yf,zf)

for i = length(cy)
    show(cy(i));
    hold on
end
show(elev)
set(gca,'Ydir','reverse');
set(gca,'DataAspectRatio',[30 10 3])
hold off
grid on
xlim([0 xf]);
ylim([0 yf]);
zlim([0 zf]);
xlabel('x [m]')
ylabel('y [m]')
zlabel('z [m]')
title('Restrictions in the flight domain')
view(210,55)

end