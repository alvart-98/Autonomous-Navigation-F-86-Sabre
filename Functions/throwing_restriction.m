function thro = throwing_restriction(xf,yf,zf,alt_imp,a1)

d1 = (zf-alt_imp)/tand(a1);

points = [xf yf zf;xf yf alt_imp];

% Circle
for y = linspace(yf,(yf-d1),500)
    x = real(xf-sqrt(d1^2-(y-yf)^2));
    points = [points;x y zf];
end

% d1 = (zf/1000-alt_imp/1000)/tand(a1);
% 
% points = [xf/1000 yf/1000 zf;xf/1000 yf/1000 alt_imp];
% 
% % Circle
% for y = linspace(yf/1000,(yf/1000-d1),500)
%     x = xf/1000-sqrt(d1^2-(y-yf/1000)^2);
%     points = [points;x y zf];
% end
% 
% thro = collisionMesh(points);


% h1  = alt_imp+d1*tand(a1);
% d2  = d1+(zf-h1)/tand(a2);
% 
% points = [xf yf zf;xf yf alt_imp];
% 
% % Circle sup
% for y = linspace(yf,(yf-d2),500)
%     x = xf-sqrt(d2^2-(y-yf)^2);
%     points = [points;x y zf];
% end
% 
% % Circle inf
% for y = linspace(yf,(yf-d1),500)
%     x = xf-sqrt(d1^2-(y-yf)^2);
%     points = [points;x y h1];
% end
% 
thro = collisionMesh(points);

end