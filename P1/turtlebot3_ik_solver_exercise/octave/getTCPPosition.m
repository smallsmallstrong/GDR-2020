%%-----------------------------------------------------------------
%% 
%%-- The method getTCPPosition extracts the endeffector position
%%   with regard to the base from the data structure of the
%%   manipulators transformations resulting form forwardKinematicsDH.
%%-----------------------------------------------------------------
%%-- INPUT
%%   TM      [1xN]  Data structure of all transformation matrices
%%    .T     [4x4]  Transformation matrix of the i-th robot link with regard to the base
%%
%%-- OUTPUT
%%   p       [3x1]  Endeffector possition with regard to the base
%%-----------------------------------------------------------------
function p = getTCPPosition(TM)
  % initialize return value
  p = zeros(3,1);

  % | Implement your code here |
  % v                          v



  % ^                          ^
  % | -------- End ----------- |
end

