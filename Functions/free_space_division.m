function [cb,free_cbs] = free_space_division(max_divisions,xf,yf,zf,cy,elev,thro,Zmax,nv,Mcy)

%% DOMAIN DEFINITION

cb(:,:,1) = [xf 0  zf;... % UNW
             xf yf zf;... % UNE
             0  yf zf;... % USE
             0  0  zf;... % USW
             xf 0  0 ;... % DNW
             xf yf 0 ;... % DNE
             0  yf 0 ;... % DSE
             0  0  0 ];   % DSW
         
bx(1)      = collisionBox(xf/2^0,yf/2^0,zf/2^0);
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
     [cls,ins] = cls_bx_obj(bx(slct),cy,elev,thro,Zmax,cb(:,:,slct),nv,Mcy);
    if slct == position0
        position0 = position+1;
    end
    if (cls == 1)&&(ins == 0)
       vs = vertix_div(cb(:,:,slct));
     for j = 1:8
         cb(:,:,position+j) = vs(:,:,j);
         bx(position+j) = collisionBox(xf/2^i,yf/2^i,zf/2^i);
         cbcen = trvec2tform([(cb(1,1,position+j)+cb(8,1,position+j))/2 (cb(2,2,position+j)...
                 +cb(1,2,position+j))/2 (cb(1,3,position+j)+cb(8,3,position+j))/2]);
         bx(position+j).Pose = cbcen;
     end
       position  = position+8;
    elseif (cls == 0)&&(ins == 0)
       free_cbs = [free_cbs slct];
    end
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

end
