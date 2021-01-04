function cls = cls_esf_obj(esf,c,cy,elev,thro,nv,hmax)

cls  = 0;

% Checking of collision with obstacles

if cls == 0
    for i = 1:length(cy)
    [cls,~,~] = checkCollision(esf,cy(i));
        if cls == 1
            return;
        end
    end
end

if cls == 0
    [cls,~,~] = checkCollision(esf,thro);
end

% nv   = [n nx ny nC];
if (cls == 0)&&(hmax >= c(3))
    p = [];
    for q = floor(abs(c(1))/nv(2)):min(ceil(abs(c(1))/nv(2))-1,nv(4)/nv(1)-2)
        for w = (floor(abs(c(2))/nv(3))+1):min(ceil(abs(c(2))/nv(3)),nv(1)-1)
            p = [p (nv(1)-1)*q+w];
        end
    end
    for j = p
    [cls,~,~] = checkCollision(esf,elev(j));
        if cls == 1
            break;
        end
    end
end

end