function [] = terrain_elevation(xf,yf,C,Z,n)

x = linspace(0,xf,length(C)/n);
y = transpose(C(1:n,2));
[X,Y] = meshgrid(x,y);

surf(X,Y,Z)
set(gca,'Ydir','reverse');
grid on
xlim([0 xf]);
ylim([0 yf]);
% zlim([0 max(Z(:))]);
xlabel('x [m]','FontSize',12,'FontWeight','bold')
ylabel('y [m]','FontSize',12,'FontWeight','bold')
zlabel('z [m]','FontSize',12,'FontWeight','bold')
% title('Terrain elevation')
% colorbar
view(210,70)

end