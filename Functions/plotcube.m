function [] = plotcube(cb,m,op,color)

% Lateral faces
c = cb(:,:,m);
x = [c(1,1) c(1,1) c(3,1) c(3,1) c(1,1);...
    c(1,1) c(1,1) c(3,1) c(3,1) c(1,1)]/1000;
y = [c(1,2) c(2,2) c(2,2) c(1,2) c(1,2);...
    c(1,2) c(2,2) c(2,2) c(1,2) c(1,2)]/1000;
z = [c(1,3) c(1,3) c(1,3) c(1,3) c(1,3);...
    c(5,3) c(5,3) c(5,3) c(5,3) c(5,3)];
% % Top face
% xt = [c(1,1) c(1,1);...
%      c(3,1) c(3,1)];
% yt = [c(1,2) c(2,2);...
%      c(1,2) c(2,2)];
% zt = [c(1,3) c(1,3);...
%      c(1,3) c(1,3)];
% % Bottom face
% xb = xt;
% yb = yt;
% zb = [c(5,3) c(5,3);...
%      c(5,3) c(5,3)];

surf(x, y, z, 'FaceColor',color,'FaceAlpha',op); % Lateral faces
hold on
% surf(xt, yt, zt, 'FaceColor',[0.5 0.5 0.5],'FaceAlpha',1); % Top face
% hold on
% surf(xb, yb, zb, 'FaceColor',[0.5 0.5 0.5],'FaceAlpha',1); % Bottom face
% a1 = 0;
% b1 = 0;
% for i = f
%     [a,~] = ismember(c(1:4,:),cb(:,:,i),'rows');
%     [b,~] = ismember(c(5:8,:),cb(:,:,i),'rows');
%     if a
%         a1 = 1;
%     elseif b
%         b1 = 1;
%     end
%     if a1&b1
%         return
%     end
% end
% 
% if a1
%     patch(x(2,:)', y(2,:)', z(2,:)', [0.5 0.5 0.5],'FaceAlpha',0.9); % Make Cube Appear Solid
%     return
% elseif b1
%     patch(x(1,:)', y(1,:)', z(1,:)', [0.5 0.5 0.5],'FaceAlpha',0.9); % Make Cube Appear Solid
%     return
% end

patch(x', y', z', color,'FaceAlpha',op); % Make Cube Appear Solid

end