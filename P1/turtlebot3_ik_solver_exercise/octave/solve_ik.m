clc; clear;

debug_on_error();

load_ros;
init_ros();

fk_request_sub = subscribe("/turtlebot3/fk_request", "turtlebot3_exercise_msgs/FKRequest");
fk_result_pub = advertise("/turtlebot3/fk_result", "turtlebot3_exercise_msgs/FKResult");

ik_request_sub = subscribe("/turtlebot3/ik_request", "turtlebot3_exercise_msgs/IKRequest");
ik_result_pub = advertise("/turtlebot3/ik_result", "turtlebot3_exercise_msgs/IKResult");

base_link_name = "arm_link_1";

joint_names = ["arm_joint_1"; "arm_joint_2"; "arm_joint_3"; "arm_joint_4"];
link_names = ["arm_link_1"; "arm_link_2"; "arm_link_3"; "arm_link_4"];

dh = turtleArmDH();
q0 = [0 0 0 0]';
link_length = [dh(1).d, dh(2).a, dh(3).a, dh(4).a];


unwind_protect
  while(1)
    pause(0.1)
    
    # handle ik request
    if (fk_request_sub.hasNewMessage())
      fprintf('\n======\nReceived FK request.\n');
      
      % -= Read and convert request message
      req = msg2struct(fk_request_sub.getMessage());
      
      if (length(req.position) != 4)
        msg = turtlebot3_exercise_msgs_FKResult(req.uid, NaN, NaN, false);
        fprintf('Invalid FK request!\n');
      else
        % compute forward kinematics
        TM = forwardKinematicsDH(dh, req.position);
        
        % generate result message
        header = Header(req.uid, 0, 0, base_link_name);
        poses = [];
        
        for i=1:4
          poses = [poses PoseStamped(header, trans2pose(TM(i).T))];
        end
        
        msg = turtlebot3_exercise_msgs_FKResult(req.uid, link_names, poses, true);
      end
      
      % publish result
      fk_result_pub.publish(msg);
      
      fflush(stdout);
    end
    
    # handle ik request
    if (ik_request_sub.hasNewMessage())
      fprintf('\n======\nReceived IK request.\nStart solving...\n');
      fflush(stdout);
      
      % -= Read and convert request message
      req = msg2struct(ik_request_sub.getMessage());
      trans = quat2trans(req.goal_pose.pose.orientation.x, req.goal_pose.pose.orientation.y, req.goal_pose.pose.orientation.z, req.goal_pose.pose.orientation.w);
      request_pose = [trans [req.goal_pose.pose.position.x req.goal_pose.pose.position.y req.goal_pose.pose.position.z]'; [0 0 0 1]];
      request_position = [req.goal_pose.pose.position.x req.goal_pose.pose.position.y req.goal_pose.pose.position.z]';

      % Get seed configuration
      q0 = req.position';
      
      % Compute admissiblae pitch and yaw from requested rotation matrix
      % pitch is afterward regarded according to manipulator zero position, pointing upward
      request_tcp_direction = request_pose(1:3,1:3) * [1; 0; 0];
      
      % TODO: Move requested position out of singularity if (x,y) position is (0,0)
      yaw = atan2(request_position(2), request_position(1));
      rotation_z = [cos(-yaw), -sin(-yaw), 0; sin(-yaw), cos(-yaw), 0; 0, 0, 1];
      tcp_direction_yaw_removed = rotation_z * request_tcp_direction;
      
      % Compute pitch from projected tcp direction on x-z planerot
      pitch = atan2(tcp_direction_yaw_removed(1),tcp_direction_yaw_removed(3));
      
      
      % -= Compute inverse kinematics
      py = [pitch, yaw]'; % roll is omitted, since kinematic structure is not suitable
      l = link_length;
      qs = [];
      % get IK result
      qs = [qs; normalizeAngles(inverseKinematics(request_position, py, l))];
      % Compute solutions with flipped base rotation
      if ~isempty(qs)
        % first elbow configuration with flipped base
        qs = [qs; normalizeAngles([qs(1,1)-pi -qs(1,2:4)])];
        % second elbow configuration with flipped base
        qs = [qs; normalizeAngles([qs(2,1)-pi -qs(2,2:4)])];
      end
      
      % -= Regard angle offsets for turtle bot
      for i=1:size(qs,1)
        qs(i,2) = qs(i,2) + (pi/2 - dh(2).theta);
        qs(i,3) = qs(i,3) - dh(3).theta;
        qs(i,:) = normalizeAngles(qs(i,:));
      end
      
      % -= Show solutions
      fprintf('\nSolutions:\n');
      disp(qs);
      fprintf('\n');
      
            
      % -= Check if solution found
      solution_found = false;
      eps = 0.0;
      qs_result = [];
      if size(qs,1) > 0
        solution_found = true;
      end
      
      fprintf('\n-=Filter solutions according to seed');        
      fprintf('\nSeed:\n');
      disp(q0');
      
      % -= Filter solutions according to joint limits
      if solution_found
        % Limits taken from urdf
        lowerLimits = [-pi+0.001, -100 * pi / 180, -180 * pi / 180, -95 * pi / 180];
        upperLimits = [pi-0.001, 100 * pi / 180, 25 * pi / 180, 95 * pi / 180];
        
        qs_valid = [];
        for i=1:size(qs,1)
          limitsViolated = false;
          qs_i = qs(i,:);
          for j=1:size(qs_i,2)
            if qs_i(j) > upperLimits(j) || qs_i(j) < lowerLimits(j)
              limitsViolated = true;
            end
          end
          if ~limitsViolated
            qs_valid = [qs_valid; qs_i];
          end
        end
        fprintf('\nValid solutions according to joint limits:\n');
        disp(qs_valid);
        
        solution_found = false;
        if size(qs_valid,1) > 0
          solution_found = true;
        end
      end
      
      % -= Filter solutions according to requested seed if found
      if solution_found
        
        qs_filtered = [];
        % Filter solutions according to base rotation distance
        for i=1:size(qs_valid,1)
          if abs(qs_valid(i,1) - q0(1)) < 2*pi
            qs_filtered = [qs_filtered; qs_valid(i,:)];
          end
        end
        fprintf('\nFiltered solutions resulting in a base rotation smaller 2*pi from seed:\n');
        disp(qs_filtered);
        
        % Filter elbow configuration
        solutionIndex = 0;
        shoulderDistance = 2*pi;
        for i=1:size(qs_filtered,1)
          if abs(qs_filtered(i,2) - q0(2)) < shoulderDistance;
            shoulderDistance = abs(qs_filtered(i,2) - q0(2));
            solutionIndex = i;
          end
        end
        if solutionIndex > 0
          qs_result = qs_filtered(solutionIndex,:);
        else
          solution_found = false;
        end        
        fprintf('\nFiltered solutions according to elbow configuration:\n');
        disp(qs_result);
        
      end
      
      % -= Compute position error
      if solution_found
        pose_s = forwardKinematicsDH(dh, qs_result);   
        eps = norm(getTCPPosition(pose_s)-request_pose(1:3,4));      
        
        fprintf('\nPosition error:\n');
        disp(eps);
        fprintf('\n');
      end
      
      
      # verify result      
      if solution_found && (eps < 0.0001)        
        disp(["Sent IK solution with joint angles:\n\t" array2str(qs_result) "."]);
      else
        fprintf('No valid solution found!\n');
        if ~solution_found
          disp("Query propably out of reach?");
        else
          disp(["Position error of found configuration too large: " num2str(eps)]);
        end
      end

      msg = turtlebot3_exercise_msgs_IKResult(req.uid, joint_names, qs_result, eps, solution_found);
      ik_result_pub.publish(msg);
      
      fflush(stdout);
    end
  endwhile
unwind_protect_cleanup
  ros.disconnect();
end_unwind_protect
