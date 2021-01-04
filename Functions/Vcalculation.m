function [Vpoints,S] = Vcalculation(Vmin,Vmax,Vi,ac0,Ppath,psi,theta,i,posf)

Vimpact = 170;

V       = Vi;
Vpoints = [];
S       = [0];

for j = 1:size(Ppath,1)-1
    dis = sqrt((Ppath(j+1,1)-Ppath(j,1))^2+(Ppath(j+1,2)-Ppath(j,2))^2+(Ppath(j+1,3)-Ppath(j,3))^2);
    S   = [S dis+S(end)];
    if (abs(psi(i+2)-psi(i+1)) == 0)&&(abs(psi(i+3)-psi(i+2)) == 0)
        c = max(1-abs(theta(i+1))/5,0);
    elseif (abs(psi(i+2)-psi(i+1)) <= 10)&&(abs(psi(i+3)-psi(i+2)) <= 10)%&&(psi(i+4)-psi(i+3) == 0)
        c = 0.5*max(1-abs(theta(i+1))/5,0);
    elseif (abs(psi(i+2)-psi(i+1)) <= 90)||(abs(psi(i+3)-psi(i+2)) <= 90)||...
           (abs(psi(i+4)-psi(i+3)) <= 90)
        c = -max([min(abs(psi(i+2)-psi(i+1))/60,1),min(abs(psi(i+3)-psi(i+2))/75,1),...
            min(abs(psi(i+4)-psi(i+3))/90,1)]);
    end
    if norm(Ppath(1,1:2)-posf) <= 1.75*(Vmax-Vimpact)/ac0
        Vmin = Vimpact;
        c = -0.75;
    end
    V = V+c*ac0*dis;
    if V < Vmin
        V = Vmin;
    elseif V >Vmax
        V = Vmax;
    end
    Vpoints = [Vpoints;V];
end

S(1) = [];

end