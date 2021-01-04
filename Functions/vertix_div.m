function vs = vertix_div(rcg)

% rcg = Rectangloid FATHER, calculation of its CHILDREN

x0 = rcg(4,1);
x1 = rcg(1,1);
xm = (x0+x1)/2;
y0 = rcg(1,2);
y1 = rcg(2,2);
ym = (y0+y1)/2;
z0 = rcg(5,3);
z1 = rcg(1,3);
zm = (z0+z1)/2;

% UNW
vs(:,:,1) = [x1 y0 z1;... % UNW
             x1 ym z1;... % UNE
             xm ym z1;... % USE
             xm y0 z1;... % USW
             x1 y0 zm;... % DNW
             x1 ym zm;... % DNE
             xm ym zm;... % DSE
             xm y0 zm];   % DSW
% UNE        
vs(:,:,2) = [zeros(8,1) (ym-y0)*ones(8,1) zeros(8,1)]+vs(:,:,1);
% USE
vs(:,:,3) = [(x0-xm)*ones(8,1) (ym-y0)*ones(8,1) zeros(8,1)]+vs(:,:,1);
% USW
vs(:,:,4) = [(x0-xm)*ones(8,1) zeros(8,1) zeros(8,1)]+vs(:,:,1);
% DNW
vs(:,:,5) = [zeros(8,1) zeros(8,1) (z0-zm)*ones(8,1)]+vs(:,:,1);
% DNE
vs(:,:,6) = [zeros(8,1) (ym-y0)*ones(8,1) (z0-zm)*ones(8,1)]+vs(:,:,1);
% DSE
vs(:,:,7) = [(x0-xm)*ones(8,1) (ym-y0)*ones(8,1) (z0-zm)*ones(8,1)]+vs(:,:,1);
% DSW
vs(:,:,8) = [(x0-xm)*ones(8,1) zeros(8,1) (z0-zm)*ones(8,1)]+vs(:,:,1);

end