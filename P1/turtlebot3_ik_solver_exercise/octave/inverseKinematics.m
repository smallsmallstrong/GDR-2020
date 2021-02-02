%%-----------------------------------------------------------------
%%-- The method inverseKinematics computes the joint configurations
%%   suitable to reach the requested position and orientation in
%%   both elbow configurations.
%% 
%%   Input values are the endeffector position, the endeffector
%%   orientation with the yaw rotation about the z_0-axis and
%%   pitch angle between the z_0-axis and the endeffector direction
%%   described by x_n-axis.
%%-----------------------------------------------------------------
%%-- INPUT
%%   request_position       [3x1]  Requested endeffector position as [x, y, z]'
%%   py                     [2x1]  Requested pitch and yaw as [pitch, yaw]'
%%   l                      [Nx1]  Link lengths
%%
%%-- OUTPUT
%%   qs                     [2xN]  Resulting joint configurations or empty [] if
%%                                 no solution can be found
%%-----------------------------------------------------------------
function qs = inverseKinematics(request_position, py, l)
  % initialize return value
  qs = [];

  % | Implement your code here |
  % v                          v

  

  % ^                          ^
  % | -------- End ----------- |
end
