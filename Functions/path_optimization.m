function [pathref,Stot] = path_optimization(path,theta,psi,Vmin,Vmax,V0,ac0,ds,cy,elev,thro,nv,Zmax)

% This function applies the Smooth 3D Path Planning algorithm. The velocity
% profile is generated and defined in Vcalculation.m. In order to achieve
% proper results, change default parameters if it was needed. 

theta(end+1:end+4) = ones(1,4)*theta(end);
psi(end+1:end+4)   = ones(1,4)*psi(end);

nmax = 5.5;
n0   = 1.0750;
fctr0 = 0.05;
fctr_precision = 0.010;
fctr_v = [[1:size(path,1)-1];zeros(1,size(path,1)-1)];

solution = 0;
k_pos    = [];

while solution == 0
    try
        pathref = [path(1,:) V0];
        Stot    = [0];
        svec    = [];
        i       = 1;
        flag    = 0;
        cls     = 0;
while i ~= size(path,1)
    if ((psi(i+2)-psi(i+1)) ~= 0)||((theta(i+2)-theta(i+1)) ~= 0)
        v1  = [path(i,1)-path(i+1,1) path(i,2)-path(i+1,2) path(i,3)-path(i+1,3)];
        v2  = [path(i+2,1)-path(i+1,1) path(i+2,2)-path(i+1,2) path(i+2,3)-path(i+1,3)];
        v12 = cross(v1,v2);
        alpha = atan2d(norm(v12),dot(v1,v2));
        v1  = v1./norm(v1);
        v2  = v2./norm(v2);
        v12 = v12./norm(v12);
        n   = n0+(fctr0+fctr_v(2,i))*abs(psi(i+2)-psi(i+1))/90;
        R   = pathref(end,4)^2/(9.81*sqrt(n^2-1));
        if R < pathref(end,4)^2/(9.81*sqrt(nmax^2-1))
            R = pathref(end,4)^2/(9.81*sqrt(nmax^2-1));
        end
        d   = R*sind((180-alpha)/2);
        s   = d/sind(alpha/2);
        svec = [svec [s;i]];
        r1  = cross(v12,v1);
        r1  = r1./norm(r1);
        r2  = cross(v2,v12);
        r2  = r2./norm(r2);
        s1  = s_point(v1,path(i+1,:),s,psi(i+1));
        s2  = s_point(v2,path(i+1,:),s,psi(i+2));
        O   = center(s1,s2,r1,r2,v1,v2);
        pc  = circle_points(v12,s1,s2,O,R,ds);
        if ~flag
            pL1 = line_points(path(i,:),s1,v1,ds);
            [V,S]   = Vcalculation(Vmin,Vmax,pathref(end,4),ac0,[pathref(end,1:3);pL1;pc],psi,theta,i,path(end,1:2));
            pathref = [pathref;[pL1;pc] V];
            Stot    = [Stot S+Stot(end)];
        else
            pL3 = line_points(pathref(end,:),s1,v1,ds);
            [V,S]   = Vcalculation(Vmin,Vmax,pathref(end,4),ac0,[pathref(end,1:3);pL3;pc],psi,theta,i,path(end,1:2));
            pathref = [pathref;[pL3;pc] V];
            Stot    = [Stot S+Stot(end)];
        end
        flag = 1;
        if (((psi(i+3)-psi(i+2)) == 0)&&((theta(i+3)-theta(i+2)) == 0))%||(i == size(path,1)-2)
            pL2 = line_points(s2,path(i+2,:),v2,ds);
            [V,S]   = Vcalculation(Vmin,Vmax,pathref(end,4),ac0,[pathref(end,1:3);pL2;path(i+2,:)],psi,theta,i,path(end,1:2));
            pathref = [pathref;[pL2;path(i+2,:)] V];
            Stot    = [Stot S+Stot(end)];
            i = i+1;
            flag = 0;
        end
        i = i+1;
    else
        v1 = [path(i+1,1)-path(i,1) path(i+1,2)-path(i,2) path(i+1,3)-path(i,3)];
        pL = line_points(path(i,:),path(i+1,:),v1,ds);
        [V,S]   = Vcalculation(Vmin,Vmax,pathref(end,4),ac0,[pathref(end,1:3);pL;path(i+1,:)],psi,theta,i,path(end,1:2));
        pathref = [pathref;[pL;path(i+1,:)] V];
        Stot    = [Stot S+Stot(end)];
        i = i+1;
        flag = 0;
    end
end
       k_pos   = [];
       for k = 1:size(svec,2)-1
           pos_i = svec(2,k);
           dis1 = sqrt((path(pos_i+1,1)-path(pos_i,1))^2+(path(pos_i+1,2)...
                  -path(pos_i,2))^2+(path(pos_i+1,3)-path(pos_i,3))^2);
           dis2 = sqrt((path(pos_i+2,1)-path(pos_i+1,1))^2+(path(pos_i+2,2)...
                  -path(pos_i+1,2))^2+(path(pos_i+2,3)-path(pos_i+1,3))^2);
           if ((svec(2,k+1) == pos_i+1)&&(svec(1,k+1)+svec(1,k) > dis2))
               k_pos = [k_pos svec(2,k) svec(2,k+1)];
           elseif ((svec(2,k+1) ~= pos_i+1)&&(svec(1,k) > dis2))||(svec(1,k) > dis1)
               k_pos = [k_pos svec(2,k)];
           end
       end
            dis_i = sqrt((path(2,1)-path(1,1))^2+(path(2,2)...
                    -path(1,2))^2+(path(2,3)-path(1,3))^2);
            dis_end = sqrt((path(end,1)-path(end-1,1))^2+(path(end,2)...
                      -path(end-1,2))^2+(path(end,3)-path(end-1,3))^2);  
       if (svec(1,1) > dis_i)
           k_pos = [k_pos 1];
       elseif (svec(1,end) > dis_end)
           k_pos = [k_pos size(svec,2)];
       end
       if length(k_pos) > 0
           error;
       end
       
          esf = collisionSphere(1);
       for w = 1:size(pathref,1)-3500/ds
          esfc = trvec2tform(pathref(w,1:3));
          esf.Pose = esfc;
          cls = cls_esf_obj(esf,pathref(w,1:3),cy,elev,thro,nv,Zmax);
          if cls
              error;
          end
       end
       solution = 1;
       
    catch
        if ~cls
            for h = k_pos
                fctr_v(2,h) = fctr_v(2,h)+fctr_precision;
            end
        else
            fctr_v(2,:) = fctr_v(2,:)+fctr_precision;
            fctr0       = fctr0+fctr_precision;
            n0          = n0+fctr_precision;
        end
    end
end

PSI  = [];
dpsi = [];
for m = 1:size(pathref,1)-2
    t1 = sqrt((pathref(m+1,1)-pathref(m,1))^2+(pathref(m+1,2)-pathref(m,2))^2+...
         (pathref(m+1,3)-pathref(m,3))^2)/pathref(m,4)/2;
    t2 = sqrt((pathref(m+2,1)-pathref(m+1,1))^2+(pathref(m+2,2)-pathref(m+1,2))^2+...
         (pathref(m+2,3)-pathref(m+1,3))^2)/pathref(m+1,4)/2;
    t     = t1+t2;
    ps1   = atan2d(pathref(m+1,2)-pathref(m,2),pathref(m+1,1)-pathref(m,1));
    ps2   = atan2d(pathref(m+2,2)-pathref(m+1,2),pathref(m+2,1)-pathref(m+1,1));
    psdif = ps2-ps1;
    PSI   = [PSI;ps1];
    dpsi  = [dpsi;psdif/t];
end
PSI  = [PSI;ps2;ps2];
for q = 1:size(dpsi,1)
    if abs(dpsi(q,1)) > 8
        dpsi(q,1) = 0;
    end
end
dpsi = [0;dpsi;0];
pathref = [pathref PSI dpsi];

THETA  = [];

for w = 1:size(pathref,1)-2
    tht   = atan2d(pathref(w+1,3)-pathref(w,3),sqrt((pathref(w+1,1)-...
            pathref(w,1))^2+(pathref(w+1,2)-pathref(w,2))^2));
    THETA = [THETA;tht];
end
THETA   = [THETA;tht;tht];
pathref = [pathref THETA];

end