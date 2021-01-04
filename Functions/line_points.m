function points = line_points(pi,pf,v,ds)

if v(1) ~= 0
    t1 = (pf(1)-pi(1))/v(1);
elseif v(2) ~= 0
    t1 = (pf(2)-pi(2))/v(2);
end
r  = sqrt((pi(1)-pf(1))^2+(pi(2)-pf(2))^2+(pi(3)-pf(3))^2);
n  = ceil(r/ds);
dt = t1/n;
t  = dt;
points = [];

while round(t,5) ~= round(t1,5)
    points = [points;pi(1)+v(1)*t,pi(2)+v(2)*t,pi(3)+v(3)*t];
    t = t+dt;
end

end