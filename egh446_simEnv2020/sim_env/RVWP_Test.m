%% Positive Path Angle
X0 = 2;
Y0 = 2;
X1 = 4;
Y1 = 4;
d = 1;


figure()
hold on
axis('equal')
line([X0,X1],[Y0,Y1],'Color', 'red')
scatter([X0,X1],[Y0,Y1],'r*')

%Left of track within d
robotx = 2;roboty = 3;
[solx,soly] = RVWP_find(robotx,roboty,X0,Y0,X1,Y1,d);
scatter(solx,soly, 'g*');
scatter(robotx,roboty, 'm*')
line([robotx,solx],[roboty,soly],'Color','blue')
d2track = sqrt((solx-robotx)^2 + (soly-roboty)^2)

%Right of track within d
robotx = 3;roboty = 2;
[solx,soly] = RVWP_find(robotx,roboty,X0,Y0,X1,Y1,d);
scatter(solx,soly, 'g*');
scatter(robotx,roboty, 'g*')
line([robotx,solx],[roboty,soly],'Color','blue')
d2track = sqrt((solx-robotx)^2 + (soly-roboty)^2)


%Left of track not within d
robotx = 2.5;roboty = 3.5;
[solx,soly] = RVWP_find(robotx,roboty,X0,Y0,X1,Y1,d);
scatter(solx,soly, 'g*');
scatter(robotx,roboty, 'm*')
line([robotx,solx],[roboty,soly],'Color','blue')
d2track = sqrt((solx-robotx)^2 + (soly-roboty)^2)

%Right of track not within d
robotx = 3.5;roboty = 2.5;
[solx,soly] = RVWP_find(robotx,roboty,X0,Y0,X1,Y1,d);
scatter(solx,soly, 'g*');
scatter(robotx,roboty, 'g*')
line([robotx,solx],[roboty,soly],'Color','blue')
d2track = sqrt((solx-robotx)^2 + (soly-roboty)^2)

%% Negative Path Angle
X0 = 2;
Y0 = 4;
X1 = 4;
Y1 = 2;
d = 1;


figure()
hold on

axis('equal')
line([X0,X1],[Y0,Y1],'Color', 'red')
scatter([X0,X1],[Y0,Y1],'r*')

%Left of track within d
robotx = 2.5;roboty = 3;
[solx,soly] = RVWP_find(robotx,roboty,X0,Y0,X1,Y1,d);
scatter(solx,soly, 'g*');
scatter(robotx,roboty, 'm*')
line([robotx,solx],[roboty,soly],'Color','blue')
d2track = sqrt((solx-robotx)^2 + (soly-roboty)^2)

%Right of track within d
robotx = 3;roboty = 3.5;
[solx,soly] = RVWP_find(robotx,roboty,X0,Y0,X1,Y1,d);
scatter(solx,soly, 'g*');
scatter(robotx,roboty, 'g*')
line([robotx,solx],[roboty,soly],'Color','blue')
d2track = sqrt((solx-robotx)^2 + (soly-roboty)^2)


%Left of track not within d
robotx = 2.5;roboty = 2;
[solx,soly] = RVWP_find(robotx,roboty,X0,Y0,X1,Y1,d);
scatter(solx,soly, 'g*');
scatter(robotx,roboty, 'm*')
line([robotx,solx],[roboty,soly],'Color','blue')
d2track = sqrt((solx-robotx)^2 + (soly-roboty)^2)

%Right of track not within d
robotx = 3.5;roboty = 4;
[solx,soly] = RVWP_find(robotx,roboty,X0,Y0,X1,Y1,d);
scatter(solx,soly, 'g*');
scatter(robotx,roboty, 'g*')
line([robotx,solx],[roboty,soly],'Color','blue')
d2track = sqrt((solx-robotx)^2 + (soly-roboty)^2)

%% Zero Path Angle
X0 = 2;
Y0 = 2;
X1 = 4;
Y1 = 2;
d = 0.5;


figure()
hold on
axis('equal')
line([X0,X1],[Y0,Y1],'Color', 'red')
scatter([X0,X1],[Y0,Y1],'r*')


for roboty = 1:0.5:3
    
    robotx = 3;
    [solx,soly] = RVWP_find(robotx,roboty,X0,Y0,X1,Y1,d);
    scatter(solx,soly, 'g*');
    scatter(robotx,roboty, 'm*')
    line([robotx,solx],[roboty,soly],'Color','blue')
    d2track = sqrt((solx-robotx)^2 + (soly-roboty)^2)
end

%% Zero Path Angle
X0 = 2;
Y0 = 2;
X1 = 2;
Y1 = 4;
d = 0.5;


figure()
hold on
axis('equal')
line([X0,X1],[Y0,Y1],'Color', 'red')
scatter([X0,X1],[Y0,Y1],'r*')


for robotx = 1:0.5:3
    
    roboty = 3;
    [solx,soly] = RVWP_find(robotx,roboty,X0,Y0,X1,Y1,d);
    scatter(solx,soly, 'g*');
    scatter(robotx,roboty, 'm*')
    line([robotx,solx],[roboty,soly],'Color','blue')
    d2track = sqrt((solx-robotx)^2 + (soly-roboty)^2)
end