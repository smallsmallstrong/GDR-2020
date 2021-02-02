clc; clear;
format long

test_tcp_pos = true;
test_dir_kin = true;
test_inv_kin = true;

% ======
% - Test cases for getTCPPosition
if test_tcp_pos
  TM = struct ('T', [2 3 4 5; 4 6 8 10; 6 9 12 15; 0 0 0 1]);
  expected = [5 10 15]';
  position_error = norm(expected - getTCPPosition(TM));
  fprintf('Get TCP position test\nError:\n');
  disp(position_error);
end

% ======
% - Test cases for forwardKinematicsDH
% Create robot DH-Description
dh(1).theta = pi / 2; dh(1).d = 0; dh(1).a = 1; dh(1).alpha = 0; dh(1).rho = 1;
dh(2).theta = 0; dh(2).d = 0; dh(2).a = 1; dh(2).alpha = 0; dh(2).rho = 1;
dh(3).theta = 0; dh(3).d = 0; dh(3).a = 0; dh(3).alpha = 0; dh(3).rho = 0;

if test_dir_kin
  % 1st test
  q = [-pi/2 pi/2 1]';
  expected_TCP_Position = [1 1 1]';
  TM = forwardKinematicsDH(dh, q);
  position_error = norm(expected_TCP_Position - getTCPPosition(TM));
  fprintf('\nForward kinematics 1st test\nposition error:\n');
  disp(position_error);

  % 2nd test
  dh(3).d = 1;
  q = [-pi/2 pi/2 0]';
  expected_TCP_Position = [1 1 1]';
  TM = forwardKinematicsDH(dh, q);
  position_error = norm(expected_TCP_Position - getTCPPosition(TM));
  fprintf('\nForward kinematics 2nd test\nposition error:\n');
  disp(position_error);

  % 3rd test
  dh = turtleArmDH();
  q = [0 0 0 0]';
  expected_TCP_Position = [2.39985710725893e-02 2.68177016862059e-17 5.14466305140020e-01]';
  TM = forwardKinematicsDH(dh, q);
  position_error = norm(expected_TCP_Position - getTCPPosition(TM));
  fprintf('\nForward kinematics 3rd test\nposition error:\n');
  disp(position_error);
end
  
% ======
% - Test cases for inverseKinematics
dh = turtleArmDH();
l = [dh(1).d, dh(2).a, dh(3).a, dh(4).a];
turtle_joint_offests = [0, (pi/2 - dh(2).theta), - dh(3).theta, 0];

if test_inv_kin
  % 1st test
  % q = [0 pi/4 pi/4 pi/4]'; % Test joint configuration
  request_position = [-3.36652257913489e-01 1.38399902569693e-18 9.91024193532457e-02]';
  py = [-3*pi/4 0]';
  qs = inverseKinematics(request_position, py, l);
  
  if size(qs,1) ~= 2 ||size(qs,2) ~= length(dh)
    fprintf('\nInverse kinematics result has wrong dimensions!\n');
    return;
  end

  % Regard angle offsets for turtle bot
  qs(1,:) = normalizeAngles(qs(1,:) + turtle_joint_offests);
  qs(2,:) = normalizeAngles(qs(2,:) + turtle_joint_offests);
  
  TM = forwardKinematicsDH(dh, qs(1,:));
  position_error_one = norm(request_position - getTCPPosition(TM));
  TM = forwardKinematicsDH(dh, qs(2,:));
  position_error_two = norm(request_position - getTCPPosition(TM));
  fprintf('\nInverse kinematics 1st test\nposition error of both elbow configurations:\n');
  disp(position_error_one);
  disp(position_error_two);
  
end
