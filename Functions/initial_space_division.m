function cbi = initial_space_division(max_divisions,xf,yf,zf)

%% DOMAIN DEFINITION

cb(:,:,1) = [xf 0  zf;... % UNW
             xf yf zf;... % UNE
             0  yf zf;... % USE
             0  0  zf;... % USW
             xf 0  0 ;... % DNW
             xf yf 0 ;... % DNE
             0  yf 0 ;... % DSE
             0  0  0 ];   % DSW

%% SPACE BUILDING

position   = 1;
position0  = 1;
slct       = 1;

for i = 1:max_divisions
 for slct = position0:position
    if slct == position0
        position0 = position+1;
    end
    vs = vertix_div(cb(:,:,slct));
    for j = 1:8
         cb(:,:,position+j) = vs(:,:,j);
    end
    position  = position+8;
 end
 if i == max_divisions
        cbi = cb(:,:,position0:position);
 end
end

end