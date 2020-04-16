function [solx,soly] = RVWP_find(robotx,roboty,X0,Y0,X1,Y1,d)

distPrev = sqrt((robotx-X0)^2 + (roboty-Y0)^2);

pathAngle = wrapToPi(atan2(Y1-Y0,X1-X0));
robotAngle = wrapToPi(atan2(roboty - Y0, robotx - X0));

dist2path = distPrev * sin(robotAngle - pathAngle);
distUpPath = sqrt(d^2 + dist2path^2) + sqrt(distPrev^2 + dist2path^2);

solx = X0 + distUpPath*cos(pathAngle);
soly = Y0 + distUpPath*sin(pathAngle);


 
% syms x y
% [solx,soly] = vpasolve(y== ((Y1-Y0)/(X1-X0)) * (x-X0) + Y0, (x-robotx)^2 + (y-roboty)^2 == d, [x,y],[X1,Y1]);
% 
% 
% 
% 
% sol_1_dist = sqrt((solx(1)-X1)^2 + (soly(1) - Y1)^2);
% sol_2_dist = sqrt((solx(2)-X1)^2 + (soly(2) - Y1)^2);
% 
% if sol_1_dist < sol_2_dist
%     solx = double(solx(1));
%     soly = double(soly(1));
% else
%     solx = double(solx(2));
%     soly = double(soly(2));
% end
