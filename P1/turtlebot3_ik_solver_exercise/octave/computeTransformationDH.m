%%-----------------------------------------------------------------
%%-- The function computeTransformationDH computes, given the structure
%%   dh containing the DH-Parameter as well as the joint value q, the
%%   transformation matrix of one robot link.
%%-----------------------------------------------------------------
%%-- INPUT
%%   dh       [1x1]  Data structure containing four DH-Parameter and one joint type
%%    .theta  [1x1]  DH-Parameter theta_i
%%    .d      [1x1]  DH-Parameter d_i
%%    .a      [1x1]  DH-Parameter a_i
%%    .alpha  [1x1]  DH-Parameter alpha_i
%%    .rho    [1x1]  Joint type (0 = prismatic, 1 = revolute)
%%   q        [1x1]  Joint variables vector in rad or m according to the joint type
%%
%%-- OUTPUT
%%   T        [4x4]  Transformation matrix of the robot link
%%-----------------------------------------------------------------
function T = computeTransformationDH(dh, q)
  % initialize return value
  T = eye(4);

  % | Implement your code here |
  % v                          v

  

  % ^                          ^
  % | -------- End ----------- |
end
