function [elev,Z,C,n] = orography_restriction(xf,yf)

elevations = readmatrix('altitudes_Alaska_ordered.xlsx');

% Alaska: Article from Journal Electronics
% originlat = 64.650532;
% originlon = -147.066893;
% destinlat = 65.830175;
% destinlon = -143.407567;

% for i=1:size(elevations,1)
%     [s,psi0,~] = orto_distazi(originlat,originlon,elevations(i,1),elevations(i,2));
%     xfinal = s*cosd(psi0);
%     yfinal = s*sind(psi0);
%     elevations(i,1:2)=[xfinal yfinal];
% end

[C,~,~]    = unique(elevations,'rows');

% Distance between initial and final points: Great Circle distance
% [s,psi0,~] = orto_distazi(originlat,originlon,destinlat,destinlon);
% 
% xfinal = s*cosd(psi0);
% yfinal = s*sind(psi0);
% precision  = 350; % Precision in the diagonal magnitude of the domain
% divisionsx = round(xfinal/precision,0);
% divisionsy = round(yfinal/precision,0);

% for i=0:divisionsx
%     C(i*(divisionsy+1)+1:(i+1)*(divisionsy+1),1)= xfinal*i/divisionsx;
% end
% 
% [C,~,~]    = unique(C,'rows');
% 
% for i=0:divisionsx
%     for j=0:divisionsy
%         C(i*(divisionsy+1)+1+j,2)= yfinal*j/divisionsy;
%     end
% end
% 
% writetable(C,'altitudes_Alaska_ordered.xlsx','Sheet',1,'Range','A1');

for i = 1:length(C)
    if C(i,1)~= C(i+1,1)
        n = i;
        break;
    end
end

for i = 1:length(C)/n
    for j = 1:n 
        Z(j,i) = C((i-1)*n+j,3);
    end
end

flag = 1;
% Security parameter
bw = 37.1*0.3048;
C1 = [C(:,1:2) C(:,3)+3*bw];
for j = 0:n:length(C1)-2*n
    for i = 1+j:n-1+j
        points = [C1(i:(i+1),:);C1((n+i):(n+i+1),:);C1(i,1:2) 0;C1(n+i,1:2) 0;...
                 C1(i+1,1:2) 0;C1(n+i+1,1:2) 0];
        elev(flag)= collisionMesh(points);
        flag = flag+1;
    end
end
% for h = 1:length(elev)
%     show(elev(h));
%     hold on
% end
% set(gca,'Ydir','reverse');
% hold off
% grid on
% xlim([0 xf]);
% ylim([0 yf]);
% zlim([0 zf]);
% xlabel('x [m]')
% ylabel('y [m]')
% zlabel('z [m]');
% set(gca,'DataAspectRatio',[30 10 3])
end