function [best,theta,psi] = rewards(ctrs,sk_ctr,qf,Mdir,Mdis,rho1,rho2,theta0,psi0,eps,eps2)

% This rewarding function determines the advantage of selecting each
% rectangloid center, while the most adequate option is chosen (best). 
% Distances between the instantaneous position and the next possible
% destination and, on the other hand, between the next possible waypoint
% and the final destination qf, are examined. The same happens when it
% comes to heading and flight path angles. The factors "eps" and "eps2",
% contribute to the value of epsilon, defining very risky regions or
% suitable low possible waypoints when danger areas or radar areas are
% being approached, respectively. The factors "rho1" and "rho2" will act 
% as weigh parameters to define the importance of each variable in each 
% segment.

    psqf  = atand((qf(2)-sk_ctr(2))/(qf(1)-sk_ctr(1)));
    amin  = min(ctrs(:,3));

for i = 1:size(ctrs,1)
    epsilon = 0;
    if length(eps) ~= 0
    for j = 1:size(eps,1)/4
        if ((ctrs(i,1) <= eps(1+(j-1)*4,1))&&(ctrs(i,1) >= eps(4*j,1)))&&...
           ((abs(ctrs(i,2)) <= abs(eps(2+(j-1)*4,2)))&&(abs(ctrs(i,2)) >= abs(eps(1+(j-1)*4,2))))
           epsilon = -3*rho2(1);
        end
    end
    end
    if length(eps2) ~= 0
       for k = 1:size(eps2,1)
           dist_r = sqrt((ctrs(i,1)-eps2(k,3))^2+(ctrs(i,2)-eps2(k,4))^2);
            if (dist_r >= eps2(k,1))&&(dist_r <= eps2(k,2))
                if (amin >= eps2(1+(k-1)*4,3))&&((ctrs(i,3) >= amin)&&(ctrs(i,3) <= amin+50))
                    epsilon = epsilon+rho2(1);
                elseif (amin < eps2(1+(k-1)*4,3))&&((ctrs(i,3) <= eps2(1+(k-1)*4,3))&&(ctrs(i,3) >= eps2(1+(k-1)*4,3)-100))
                    epsilon = epsilon+rho2(1);
                end
            end 
       end
%     for k = 1:size(eps2,1)/4
%     if ((ctrs(i,1) <= eps2(1+(k-1)*4,1))&&(ctrs(i,1) >= eps2(4*k,1)))&&((abs(ctrs(i,2))...
%        <= abs(eps2(2+(k-1)*4,2)))&&(abs(ctrs(i,2)) >= abs(eps2(1+(k-1)*4,2))))
%         if (amin >= eps2(1+(k-1)*4,3))&&((ctrs(i,3) >= amin)&&(ctrs(i,3) <= amin+50))
%             epsilon = epsilon+rho2(1);
%         elseif (amin < eps2(1+(k-1)*4,3))&&((ctrs(i,3) <= eps2(1+(k-1)*4,3))&&(ctrs(i,3) >= eps2(1+(k-1)*4,3)-100))
%             epsilon = epsilon+rho2(1);
%         end
%     end
%     end
    end
    Mdis1 = sqrt((ctrs(i,1)-sk_ctr(1))^2+(ctrs(i,2)-sk_ctr(2))^2+(ctrs(i,3)-sk_ctr(3))^2);
    Mdis2 = sqrt((ctrs(i,1)-qf(1))^2+(ctrs(i,2)-qf(2))^2+(ctrs(i,3)-qf(3))^2);
    th(i) = atand((ctrs(i,3)-sk_ctr(3))/sqrt((ctrs(i,1)-sk_ctr(1))^2+(ctrs(i,2)-sk_ctr(2))^2));
    Tth1  = tand(th(i)-theta0);
    if (Tth1 < -1)||(Tth1 > 1)
        Tth1 = -1;
    end
    if abs(th(i)) > 5
        epsilon = epsilon-200*abs(abs(th(i))-5)/15;
    end
    Tth2  = (qf(3)-ctrs(i,3))/sqrt((ctrs(i,1)-qf(1))^2+(ctrs(i,2)-qf(2))^2);
    if (Tth2 < -1)||(Tth2 > 1)
        Tth2 = -1;
    end
    ps(i) = atan2d((ctrs(i,2)-sk_ctr(2)),(ctrs(i,1)-sk_ctr(1)));
    Tps1  = tand(ps(i)-psi0);
    if abs(ps(i)-psi0) > 45
       Tps1 = -1;
       epsilon = epsilon-40*abs(ps(i)-psi0)/90;
    end
    if (Mdis < 10000)&&(abs(ps(i)-psqf) > 0)
       epsilon = epsilon-rho2(1)*abs(ps(i)-psqf)/20;
    end
    Tps2  = (qf(2)-ctrs(i,2))/(qf(1)-ctrs(i,1));
    if (Tps2 < -1)||(Tps2 > 1)
       Tps2 = -1;
    end
    D(i) = dot(rho1,(sin(pi*[Mdis1/Mdir Tth1 Tps1]+pi/2)+1)./2)+...
           dot(rho2,(sin(pi*[Mdis2/Mdir Tth2 Tps2]+pi/2)+1)./2)+epsilon;
end

b     = find(max(D)==D);
best  = ctrs(b(1),:);
theta = th(b(1));
psi   = ps(b(1));

end