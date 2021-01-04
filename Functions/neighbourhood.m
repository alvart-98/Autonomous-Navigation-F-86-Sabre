function sk1 = neighbourhood(sk,cbi,free,radar)

% Calculation of initial rectangloids that can be selected as candidates.
% Top (), bottom, left, right and diagonals subspaces are searched in all 
% cases. When the aircraft is approaching the radar area, the front bottom, 
% diagonal bottom right and bottom left rectangloids are selected to 
% increase the available rectangloids and improve the performance of the 
% algorithm 

sk1 = [];

for i = free
    [a,~] = ismember(round(cbi(1:4,:,sk)),round(cbi(5:8,:,i)),'rows');
    [b,~] = ismember(round(cbi(5:8,:,sk)),round(cbi(1:4,:,i)),'rows');
    [c,~] = ismember(round([cbi(1,:,sk);cbi(4:5,:,sk);cbi(8,:,sk)]),round([cbi(2:3,:,i);cbi(6:7,:,i)]),'rows');
    [d,~] = ismember(round([cbi(1:2,:,sk);cbi(5:6,:,sk)]),round([cbi(4,:,i);cbi(3,:,i);cbi(8,:,i);cbi(7,:,i)]),'rows');
    [e,~] = ismember(round([cbi(2:3,:,sk);cbi(6:7,:,sk)]),round([cbi(1,:,i);cbi(4,:,i);cbi(5,:,i);cbi(8,:,i)]),'rows');
    [f,~] = ismember(round([cbi(1,:,sk);cbi(5,:,sk)]),round([cbi(3,:,i);cbi(7,:,i)]),'rows');
    [g,~] = ismember(round([cbi(2,:,sk);cbi(6,:,sk)]),round([cbi(4,:,i);cbi(8,:,i)]),'rows');
    [h,~] = ismember(round([cbi(3,:,sk);cbi(7,:,sk)]),round([cbi(1,:,i);cbi(5,:,i)]),'rows');
    if a
        sk1 = [sk1 i];
    elseif b
        sk1 = [sk1 i];
    elseif c
        sk1 = [sk1 i];
    elseif d
        sk1 = [sk1 i];
    elseif e
        sk1 = [sk1 i];
    elseif f
        sk1 = [sk1 i];
    elseif g
        sk1 = [sk1 i];
    elseif h
        sk1 = [sk1 i];
    end
    if radar
       [I,~] = ismember(round(cbi(6,:,sk)),round(cbi(4,:,i)),'rows');
       [j,~] = ismember(round([cbi(6,:,sk);cbi(5,:,sk)]),round(cbi(3:4,:,i)),'rows');
       [k,~] = ismember(round(cbi(6:7,:,sk)),round([cbi(1,:,i);cbi(4,:,i)]),'rows');
       if I
          sk1 = [sk1 i];
       elseif j
           sk1 = [sk1 i];
       elseif k
           sk1 = [sk1 i];
       end
    end
end

end