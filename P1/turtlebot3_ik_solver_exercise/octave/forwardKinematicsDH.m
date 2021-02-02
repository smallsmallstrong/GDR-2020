%%-----------------------------------------------------------------
%%-- The method forwardKinematicsDH computes the transformation
%%   matrices, which describe the position and orientation of the
%%   joints of the manipulator with regard to the base coordinate
%%   system.
%%   Input values are the struct containing the DH-Parameter of
%%   all links of the kinematic chain and the joint variables.
%%-----------------------------------------------------------------
%%-- INPUT
%%   dh       [1xN]  Data structure containing four DH-Parameter and one joint type
%%    .theta  [1x1]  DH-Parameter theta_i
%%    .d      [1x1]  DH-Parameter d_i
%%    .a      [1x1]  DH-Parameter a_i
%%    .alpha  [1x1]  DH-Parameter alpha_i
%%    .rho    [1x1]  Joint type (0 = prismatic, 1 = revolute)
%%   q        [Nx1]  Joint variables vector in rad or m according to the joint type
%%
%%-- OUTPUT
%%   TM      [1xN]  Data structure of all transformation matrices
%%    .T     [4x4]  Transformation matrix of the i-th robot link with regard to the base
%%-----------------------------------------------------------------
function TM = forwardKinematicsDH(dh, q)
  % initialize return value
  TM = repmat(struct('T',eye(4,4)), 1, length(dh));

  % | Implement your code here |
  % v                          v

  

  % ^                          ^
  % | -------- End ----------- |
end
