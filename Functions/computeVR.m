function [VR,fpa,TASh]=computeVR(TAS,wp1,wp2)
%wp1=altitude step[ft]
%wp2=distance between two points[m]
fpa=atand((wp1/3.28084)/(wp2));
TASh=TAS*cosd(fpa);
VR=TAS*sind(fpa)*101.268591;
end