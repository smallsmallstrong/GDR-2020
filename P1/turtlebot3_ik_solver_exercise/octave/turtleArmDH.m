%%-----------------------------------------------------------------
%%-- Definition of the turtlebot's kinematic
%%-----------------------------------------------------------------
function dh = turtleArmDH()

%dh = [0       0.0405+0.036    0       pi/2    1;
%      1.41    0       0.1499  0       1;
%      0.1608  0       0.15    0       1;
%      0       0       0.14    0       1];

  dh(1).theta = 0;
  dh(1).d = 0.0405+0.036;
  dh(1).a = 0;  
  dh(1).alpha = pi/2;
  dh(1).rho = 1;  
  
  dh(2).theta = 01.41;
  dh(2).d = 0;
  dh(2).a = 0.1499;
  dh(2).alpha = 0;
  dh(2).rho = 1; 
  
  dh(3).theta = 0.1608;
  dh(3).d = 0;
  dh(3).a = 0.15;
  dh(3).alpha = 0;
  dh(3).rho = 1; 
  
  dh(4).theta = 0;
  dh(4).d = 0;
  dh(4).a = 0.14;
  dh(4).alpha = 0;
  dh(4).rho = 1; 
end