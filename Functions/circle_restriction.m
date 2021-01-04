function [cy,xc,yc,eps,eps2] = circle_restriction(c,cpsi,r,zmin,zmax,xf,yf)

% This function generates the collision objects for danger and radar areas
% while determines their centers. Moreover, the matrices "eps" and "eps2",
% which will be utilized in the calculation of epsilon parameter in the
% rewarding function,are obtained on the basis of the relative position of 
% these areas with respect to the 3D environment of study. 

% Centers of each circumference
xc = c.*cosd(cpsi);
yc = c.*sind(cpsi);

eps  = [];
eps2 = [];

% Descending start parameter for radar areas: in order that the aircraft
% can descend enough before a radar area, a pre-zone must be designed. In
% this way, points before the radar area which are at a distance "dist"
% will be benefited from the parameter "eps2" in the rewarding function. 
dist = 25000;

for i = 1:length(r)
    
    % Collision objects are created as cylinders
    cyd      = collisionCylinder(r(i),zmax(i)-zmin(i));
    T        = trvec2tform([xc(i) yc(i) (zmax(i)+zmin(i))/2]);
    cyd.Pose = T;
    cy(i)    = cyd;
    
    % The troublesome regions in which the aircraft could enter, delimited
    % by danger areas, are specified in the rewarding function by means of 
    % the "eps" matrix 
    if ((abs(yc(i))+r(i)) >= abs(yf))&&(zmin(i) == 0)
        eps = [eps;xc(i) (yf/abs(yf))*(abs(yc(i))-r(i));xc(i) yf;0 yf;0 (yf/abs(yf))*(abs(yc(i))-r(i))];
    elseif ((abs(yc(i))-r(i)) <= 0)&&(zmin(i) == 0)
        eps = [eps;xc(i) 0;xc(i) (yf/abs(yf))*(abs(yc(i))+r(i));0 (yf/abs(yf))*(abs(yc(i))+r(i));0 0];
    elseif ((abs(xc(i))+r(i)) >= abs(xf))&&(zmin(i) == 0)
        eps = [eps;xf 0;xf yc(i);xc(i)-r(i) yc(i);xc(i)-r(i) 0];
    elseif ((abs(xc(i))-r(i)) <= 0)&&(zmin(i) == 0)
        eps = [eps;xc(i)+r(i) 0;xc(i)+r(i) yc(i);0 yc(i);0 0];
    end
    
    % "eps2" matrix defines the area in which the lower allowed centers of 
    % subspaces are more suitable to fulfill the constraint of the radar zone. 
    if zmin(i) > 0
        eps2 = [eps;r(i)-dist r(i)+dist xc(i) yc(i)];
    end
%     if zmin(i) > 0
%         xcut = min(xc(i)+sqrt(r(i)^2-(yf-yc(i))^2),xc(i)-sqrt(r(i)^2-(yf-yc(i))^2));
%         ycutx = yf;
%         ycut = min(yc(i)+sqrt(r(i)^2-(xf-xc(i))^2),yc(i)-sqrt(r(i)^2-(xf-xc(i))^2));
%         xcuty = xf;
%         eps2 = [xcuty ycut-dist zmin(i);xcuty ycutx zmin(i);max(xcut-dist,0) ycutx zmin(i);max(xcut-dist,0) ycut-dist zmin(i)];
%     end
    
%     if zmin(i) > 0
%         ycut = yc(i)+sqrt(r(i)^2-(xf-xc(i))^2);
%         yCUT = max([ycut,yc(i)+r(i),yc(i)-r(i)]);
%         if xc(i) > xf
%             xCUT = xc(i)-r(i)-dist;
%         elseif xc(i) <= xf
%             xCUT = 0;
%         end
%         eps2 = [xf 0 zmin(i);xf yCUT zmin(i);xCUT yCUT zmin(i);xCUT 0 zmin(i)];
%     end

    
%     if zmin(i) > 0
%         ycut = yc(i)+sqrt(r(i)^2-(xf-xc(i))^2);
%         yCUT = max([ycut,yc(i)+r(i),yc(i)-r(i)]);
%         eps2 = [xf 0 zmin(i);xf yCUT zmin(i);0 yCUT zmin(i);0 0 zmin(i)];
%     end
    
end

end
