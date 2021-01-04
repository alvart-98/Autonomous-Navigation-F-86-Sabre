function position = cube_finding(q,cb,f)

for i = f
    [a,~] = ismember(round([q(1:2);q(1:2)]),round(cb(:,1:2,i)),'rows');
    if (a)&((q(3) >= min(cb(:,3,i)))&&(q(3) <= max(cb(:,3,i))))
        position = i;
        return
    end
end

end