function [GS,heading,WCA]=computeGS(TAS,track,windSpeed,windDir)

WindDir=windDir+180;
WTAngle=track-WindDir;
WCA=asind(windSpeed*sind(WTAngle)/TAS);
heading=track+WCA;
GS=TAS*cosd(WCA)+windSpeed*cos(WTAngle);
end 