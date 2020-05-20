  function [X,Y] = fcn(state,Xd,Yd, d)
  
  persistent firstentry
  persistent WP_index
  coder.extrinsic('wrapToPi');
  coder.extrinsic('wrapTo360');
  coder.extrinsic('wrapTo2Pi');
  coder.extrinsic('RVWP_find');

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

%Extract waypoint list
Xd(WP_index);
Yd(WP_index);
X = 0;
Y = 0;
%extract vehicle state
x=state(1);
y=state(2);
theta=state(3);

%warp angle theta
theta = theta - 2*pi*floor( (theta+pi)/(2*pi) ); 

%distance to waypoint
distance_to_current_waypoint = sqrt((x - Xd(WP_index))^2 + (y - Yd(WP_index))^2);  %WP_index is index for current waypoint

if distance_to_current_waypoint> d
    [X,Y] = RVWP_find(x,y,Xd(WP_index-1),Yd(WP_index-1),Xd(WP_index),Yd(WP_index),d);
else
    X = Xd(WP_index);
    Y = Yd(WP_index);
end

% disp('--------------------------------------------------');


%Set Capture condition here: 
if distance_to_current_waypoint < 0.2  
    WP_index = WP_index+1;
else
    WP_index = WP_index;
end



% X = Xd(WP_index);
% Y = Yd(WP_index);
