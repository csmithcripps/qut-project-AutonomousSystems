  function [vel,thetaDot] = fcn(state,Xd,Yd)
  
  persistent firstentry
  persistent WP_index
  coder.extrinsic('wrapToPi');
  coder.extrinsic('wrapTo360');
  coder.extrinsic('wrapTo2Pi');

%%%%%%%
% Upon the first call we need to initialize the variables.
if isempty(firstentry)
    firstentry = 0;
end

if isempty(WP_index)
    WP_index = 0;
end
  
if firstentry == 0
    WP_index = 2;
    firstentry = 1;
end
 
if WP_index > length(Xd)
   WP_index = 2;
end 

%controller gain
k=3.9;


%Extract waypoint list
Xd(WP_index);
Yd(WP_index);

%extract vehicle state
x=state(1);
thetaDot=state(2);
theta=state(3);

%warp angle theta
theta = theta - 2*pi*floor( (theta+pi)/(2*pi) ); 


%current heading to waypoint
psi_star = atan2((Yd(WP_index) - thetaDot ) , (Xd(WP_index) - x));

%distance to waypoint
distance_to_current_waypoint = sqrt((x - Xd(WP_index))^2 + (thetaDot - Yd(WP_index))^2);  %WP_index is index for current waypoint

% assign variables
 heading = theta; 
 heading_star = psi_star; 
%compute error signal
 error =  (heading_star - heading);


% disp('--------------------------------------------------');


%Set Capture condition here: 
if distance_to_current_waypoint < 0.2  
    WP_index = WP_index+1;
else
    WP_index = WP_index;
end

%control output, wrapped to +/-pi
thetaDot = k * wrapToPi(error);
vel = 0.5