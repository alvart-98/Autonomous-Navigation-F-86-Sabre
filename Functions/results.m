function SIMOUT = results(simout)

for i=1:length(simout.x)
    if (simout.x(i) >= 0)&&(simout.y(i) >= 0)
        sim1=i;
        break;
    end
end

SIMOUT.Fuel = simout.Fuel(i:end,:);
SIMOUT.V = simout.V(i:end,:);
SIMOUT.Vref = simout.Vref(i:end,:);
SIMOUT.Vwinds = simout.Vwinds(i:end,:);
SIMOUT.deltaA = simout.deltaA(i:end,:);
SIMOUT.deltaE = simout.deltaE(i:end,:);
SIMOUT.deltaP = simout.deltaP(i:end,:);
SIMOUT.deltaR = simout.deltaR(i:end,:);
SIMOUT.p = simout.p(i:end,:);
SIMOUT.phi = simout.phi(i:end,:);
SIMOUT.phiref = simout.phiref(i:end,:);
SIMOUT.psi = simout.psi(i:end,:);
SIMOUT.q = simout.q(i:end,:);
SIMOUT.r = simout.r(i:end,:);
SIMOUT.rumb = simout.rumb(i:end,:);
SIMOUT.rumbref = simout.rumbref(i:end,:);
SIMOUT.t = simout.t(i:end,:)-simout.t(i);
SIMOUT.theta = simout.theta(i:end,:);
SIMOUT.tout = simout.tout(i:end,:);
SIMOUT.u = simout.u(i:end,:);
SIMOUT.v = simout.v(i:end,:);
SIMOUT.w = simout.w(i:end,:);
SIMOUT.x = simout.x(i:end,:);
SIMOUT.y = simout.y(i:end,:);
SIMOUT.z = simout.z(i:end,:);
SIMOUT.zref = simout.zref(i:end,:);

end