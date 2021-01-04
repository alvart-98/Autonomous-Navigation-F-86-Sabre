function points = circle_points(v12,s1,s2,O,R,ds)

points = s1;

psi1 = real(acosd((s1(3)-O(3))/R));
psi2 = real(acosd((s2(3)-O(3))/R));
if (round(s1(2)-O(2),3) == 0)&&(round(s1(1)-O(1),3) == 0)
    theta2 = atan2d(round(s2(2)-O(2),3),round(s2(1)-O(1),3));
    theta1 = theta2;
elseif (round(s2(2)-O(2),3) == 0)&&(round(s2(1)-O(1),3) == 0)
    theta1 = atan2d(round(s1(2)-O(2),3),round(s1(1)-O(1),3));
    theta2 = theta1;
else
    theta1 = atan2d(round(s1(2)-O(2),3),round(s1(1)-O(1),3));
    theta2 = atan2d(round(s2(2)-O(2),3),round(s2(1)-O(1),3));
end

if norm(s1-s2) < 5*ds
    points = [s1;s2];
    return;
end

if (psi1 ~= psi2)&&(abs(theta1) ~= abs(theta2))
    n = ceil(abs(theta2-theta1)*(pi/180)*R/ds);
    dth = (theta2-theta1)/n;
    th = theta1;
    while round(th,5) ~= round(theta2,5)
        th = th+dth;
        vc = (v12(1)*cosd(th)+v12(2)*sind(th));
        if (sign(-v12(3)) == 1)&&(sign(vc) == 1)
            psi = atand(-v12(3)/vc);
        elseif (sign(-v12(3)) == -1)&&(sign(vc) == -1)
            psi = atand(-v12(3)/vc);
        elseif (sign(-v12(3)) == -1)&&(sign(vc) == 1)
            psi = atan2d(v12(3),-vc);
        else
            psi = atan2d(-v12(3),vc);
        end
        points = [points;R*sind(psi)*cosd(th)+O(1),R*sind(psi)*sind(th)+O(2),R*cosd(psi)+O(3)];
    end
elseif psi1 == psi2
    n = ceil(abs(theta2-theta1)*(pi/180)*R/ds);
    dth = (theta2-theta1)/n;
    th = theta1;
    while round(th,5) ~= round(theta2,5)
        th = th+dth;
        points = [points;R*sind(psi1)*cosd(th)+O(1),R*sind(psi1)*sind(th)+O(2),R*cosd(psi1)+O(3)];
    end
elseif abs(theta1) == abs(theta2)
    if sign(theta1) == sign(theta2)
        n = ceil(abs(psi2-psi1)*(pi/180)*R/ds);
        dpsi = (psi2-psi1)/n;
    else 
        n = ceil(abs(psi2+psi1)*(pi/180)*R/ds);
        dpsi = (psi2+psi1)/n;
    end
    psi = psi1;
    flag = 0;
    while round(psi,5) ~= round(psi2,5)
        if psi == 0
            flag = 1;
        end
        if sign(theta1) == sign(theta2)
            psi = psi+dpsi;
        elseif ~flag
            psi = psi-dpsi;
        elseif flag
            psi = psi+dpsi;
        end
        if ~flag
            points = [points;R*sind(psi)*cosd(theta1)+O(1),R*sind(psi)*sind(theta1)+O(2),R*cosd(psi)+O(3)];
        elseif flag
            points = [points;R*sind(psi)*cosd(theta2)+O(1),R*sind(psi)*sind(theta2)+O(2),R*cosd(psi)+O(3)];
        end
    end
end

end