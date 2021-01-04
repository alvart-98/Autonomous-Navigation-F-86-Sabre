function [] = free_space(positions,cb,xf,yf,zf,op,color)

for m = positions
    plotcube(cb,m,op,color);
    hold on
end
set(gca,'Ydir','reverse');
hold off
grid on
xlim([0 xf/1000]);
ylim([0 yf/1000]);
zlim([0 zf]);
xlabel('x [km]','FontSize',12,'FontWeight','bold')
ylabel('y [km]','FontSize',12,'FontWeight','bold')
zlabel('z [m]','FontSize',12,'FontWeight','bold')
% title('Cubic decomposition of the free space')
view(52.5,55)

end