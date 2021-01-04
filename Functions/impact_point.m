function [deviation,OPTIMAL,prmts] = impact_point(C,Z,n,xf,yf,pathref,wind,T,color)

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
    prmts = bomb_throwing2(pathref(end-i,:),wind(size(pathref,1)-i:end,2:4),x,y,Z,T);
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
OPTIMAL = size(pathref,1)-(i_initial-MINpos(1)+1);


figure()
for i = 1:size(pos_v,1)
    plot3(transpose(prmts_v(pos_v(i,1):pos_v(i,2),9))/1000,...
       transpose(prmts_v(pos_v(i,1):pos_v(i,2),10))/1000,transpose...
       (prmts_v(pos_v(i,1):pos_v(i,2),11)),'r-.','linewidth',0.25);
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
plot3(transpose(pathref(:,1))/1000,transpose(pathref(:,2))/1000,transpose(pathref(:,3)),color,'linewidth',1.5);
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
zlim([min([Z(end,end);prmts_v(:,11)]) max(prmts_v(:,11))]);
%legend('Reference Path','3D Smooth Path');
grid on
hold off

prmts     = bomb_throwing2(pathref(OPTIMAL,:),wind(OPTIMAL:end,2:4),x,y,Z,T);
deviation = sqrt((prmts(end,9)-xf)^2+(prmts(end,10)-yf)^2);


end