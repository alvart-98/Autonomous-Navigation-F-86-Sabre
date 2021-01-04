function ctrs = free_space_MACD(sk_ctr,cb0,psi,max_divisions,cy,elev,thro,Zmax,nv,Mcy)

cb(:,:,1) = cb0;
xf = abs(cb(1,1,1)-cb(4,1,1));
yf = abs(cb(1,2,1)-cb(2,2,1));
zf = abs(cb(1,3,1)-cb(5,3,1));
        
bx(1)      = collisionBox(xf,yf,zf);
cbcen      = trvec2tform([(cb(1,1,1)+cb(8,1,1))/2 (cb(2,2,1)...
             +cb(1,2,1))/2 (cb(1,3,1)+cb(8,3,1))/2]);
bx(1).Pose = cbcen;

%% SPACE BUILDING

position   = 1;
position0  = 1;
slct       = 1;
free_cbs   = [];

for i = 1:max_divisions
 for slct = position0:position
    if slct == position0
        position0 = position+1;
    end
       vs = vertix_div(cb(:,:,slct));
     for j = 1:8
         cb(:,:,position+j) = vs(:,:,j);
         bx(position+j) = collisionBox(xf/2^i,yf/2^i,zf/2^i);
         cbcen = trvec2tform([(cb(1,1,position+j)+cb(8,1,position+j))/2 (cb(2,2,position+j)...
                 +cb(1,2,position+j))/2 (cb(1,3,position+j)+cb(8,3,position+j))/2]);
         bx(position+j).Pose = cbcen;
     end
       position  = position+8;
 end
 if i == max_divisions
    for slct = position0:position
        [cls,~] = cls_bx_obj(bx(slct),cy,elev,thro,Zmax,cb(:,:,slct),nv,Mcy);
       if cls == 0
          free_cbs = [free_cbs slct]; 
       end
    end
 end
end

flag = 1;
ctrs = [];
for h = free_cbs
    %cb(:,:,flag) = cb(:,:,h); 
    alt = tand(20)*sqrt((bx(h).Pose(1,4)-sk_ctr(1))^2+(bx(h).Pose(2,4)-sk_ctr(2))^2);
    if (psi < 90)&&(psi > -90)
        if (((sk_ctr(1) <= bx(h).Pose(1,4))||((sk_ctr(1) >= bx(h).Pose(1,4))...
           &&(abs(sk_ctr(2)) <= abs(bx(h).Pose(2,4)))))&&(bx(h).Pose(1,4)...
           >= (tand(-psi)*(bx(h).Pose(2,4)-sk_ctr(2))+sk_ctr(1))))&&((bx(h).Pose(3,4) >= sk_ctr(3)-alt)&&(bx(h).Pose(3,4) <= sk_ctr(3)+alt))
            
           ctrs(flag,:) = transpose(bx(h).Pose(1:3,4));
           flag = flag+1;
        end
    elseif (psi > 90)&&(psi < -90)
        if (((sk_ctr(1) <= bx(h).Pose(1,4))||((sk_ctr(1) >= bx(h).Pose(1,4))...
           &&(abs(sk_ctr(2)) <= abs(bx(h).Pose(2,4)))))&&(bx(h).Pose(1,4)...
           <= (tand(-psi)*(bx(h).Pose(2,4)-sk_ctr(2))+sk_ctr(1))))&&((bx(h).Pose(3,4) >= sk_ctr(3)-alt)&&(bx(h).Pose(3,4) <= sk_ctr(3)+alt))
            
           ctrs(flag,:) = transpose(bx(h).Pose(1:3,4));
           flag = flag+1;
        end
    elseif (psi == 90)
        if (((sk_ctr(1) <= bx(h).Pose(1,4))||((sk_ctr(1) >= bx(h).Pose(1,4))...
           &&(abs(sk_ctr(2)) <= abs(bx(h).Pose(2,4)))))&&(abs(bx(h).Pose(2,4))...
           >= abs(sk_ctr(2))))&&((bx(h).Pose(3,4) >= sk_ctr(3)-alt)&&(bx(h).Pose(3,4) <= sk_ctr(3)+alt))
            
           ctrs(flag,:) = transpose(bx(h).Pose(1:3,4));
           flag = flag+1;
        end
    elseif (psi == -90)
        if (((sk_ctr(1) <= bx(h).Pose(1,4))||((sk_ctr(1) >= bx(h).Pose(1,4))...
           &&(abs(sk_ctr(2)) <= abs(bx(h).Pose(2,4)))))&&(abs(bx(h).Pose(2,4))...
           <= abs(sk_ctr(2))))&&((bx(h).Pose(3,4) >= sk_ctr(3)-alt)&&(bx(h).Pose(3,4) <= sk_ctr(3)+alt))
            
           ctrs(flag,:) = transpose(bx(h).Pose(1:3,4));
           flag = flag+1;
        end
    end
end

end