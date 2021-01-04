function [cls,ins] = cls_bx_obj(bx,cy,elev,thro,hmax,v,nv,Mcy)

% Check if the rectangloid is inside of the obstacle

cls  = 0;
ins  = 0;

% Mcy = [xc;yc;r;zmin;zmax];
% Mth = [alt_imp a1 a2 d1 d2 h1];

for k = 1:length(Mcy(1,:))
    g = 0;
    for u = 1:8
        if ((abs(-sqrt(Mcy(3,k)^2-(v(u,1)-Mcy(1,k))^2)+Mcy(2,k))<=abs(v(u,2)))...
           &&(abs(v(u,2))<=abs(sqrt(Mcy(3,k)^2-(v(u,1)-Mcy(1,k))^2)+Mcy(2,k))))...
           &&((v(u,3)>=Mcy(4,k))&&(v(u,3)<=Mcy(5,k)))
            
           g = g+1;
        end
    end
    if g == 8
        ins = 1;
        cls = 1;
        return
    elseif (g < 8)&&(g > 0)
        ins = 0;
        cls = 1;
    elseif (g == 0)&&(cls == 1)
        ins = 0;
        cls = 1;
    end

end

% Checking of collision with obstacles

if cls == 0
    for i = 1:length(cy)
    [cls,~,~] = checkCollision(bx,cy(i));
        if cls == 1
            return;
        end
    end
end

if cls == 0
    [cls,~,~] = checkCollision(bx,thro);
end

% nv   = [n nx ny nC];
if (cls == 0)&&(hmax >= v(5,3))
    p = [];
    for q = floor(abs(v(3,1))/nv(2)):min(ceil(abs(v(1,1))/nv(2))-1,nv(4)/nv(1)-2)
        for w = (floor(abs(v(1,2))/nv(3))+1):min(ceil(abs(v(3,2))/nv(3)),nv(1)-1)
            p = [p (nv(1)-1)*q+w];
        end
    end
    for j = p
    [cls,~,~] = checkCollision(bx,elev(j));
        if cls == 1
            break;
        end
    end
end

end