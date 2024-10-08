clear all
close all
clf
clc;

%% Plot the environment
RobotClass.Plot_Environment()
axis manual;
hold on
%% Plot the LinearUR3e and the gripper
% Call the LinearUR3e
r = LinearUR3e;
hold on 

% Take the base of end-effector 
qNow = r.model.getpos();
base = r.model.fkineUTS(qNow);

%Call the gripper with 2 fingers and plot the calculated base 
finger1 = LinearFinger(base*trotx(pi/2)); 
finger2 = LinearFinger(base*troty(pi)*trotx(-pi/2));
%% Plot the Working volume
 Volume_Plot = RobotClass.Plot_Volume(r);
 disp('Press enter to remove plot and continue');
 pause;
delete(Volume_Plot)
%% Plot 9 bricks
[b1, b2, b3, b4, b5, b6, b7, b8, b9] = RobotClass.Plot_Brick();
%% Move to first brick
RobotClass.Move_Gripper(r,finger1, finger2, RobotClass.Grip_open);
qTraj = RobotClass.Create_Trajectory(r, RobotClass.b_1_Gripper_Pos, RobotClass.elbow_Pos_Pick);
RobotClass.Move_Robot(r,qTraj,finger1,finger2);

%% Grab the 1st brick and place it down in new position
RobotClass.Move_Gripper(r, finger1, finger2, RobotClass.Grip_closed);
qTraj = RobotClass.Create_Trajectory(r, RobotClass.b_1_Place_Pos,RobotClass.elbow_Pos_Place);%Delete brick 1
RobotClass.Delete_Brick(b1);
RobotClass.Move_Brick(r,qTraj,finger1,finger2,RobotClass.b_1);

%% Move to 2nd brick position after dropping 1st brick
RobotClass.Move_Gripper(r,finger1, finger2, RobotClass.Grip_open);
qTraj = RobotClass.Create_Trajectory(r, RobotClass.b_2_Gripper_Pos, RobotClass.elbow_Pos_Pick);
RobotClass.Move_Robot(r,qTraj,finger1,finger2);

%% Grab the 2nd brick and place it down in new position
RobotClass.Move_Gripper(r,finger1, finger2, RobotClass.Grip_closed);
qTraj = RobotClass.Create_Trajectory(r, RobotClass.b_2_Place_Pos, RobotClass.elbow_Pos_Place);
RobotClass.Delete_Brick(b2);
RobotClass.Move_Brick(r,qTraj,finger1,finger2,RobotClass.b_2);

%% Move to 3rd brick position after dropping 2nd brick
RobotClass.Move_Gripper(r,finger1, finger2, RobotClass.Grip_open);
qTraj = RobotClass.Create_Trajectory(r, RobotClass.b_3_Gripper_Pos, RobotClass.elbow_Pos_Pick);
RobotClass.Move_Robot(r,qTraj,finger1,finger2);

%% Grab the 3rd brick and place it down in new position
RobotClass.Move_Gripper(r,finger1, finger2, RobotClass.Grip_closed);
qTraj = RobotClass.Create_Trajectory(r, RobotClass.b_3_Place_Pos, RobotClass.elbow_Pos_Place);
RobotClass.Delete_Brick(b3);
RobotClass.Move_Brick(r,qTraj,finger1,finger2,RobotClass.b_3);

%% Move to 4th brick position after dropping 3rd brick
RobotClass.Move_Gripper(r,finger1, finger2, RobotClass.Grip_open);
qTraj = RobotClass.Create_Trajectory(r, RobotClass.b_4_Gripper_Pos, RobotClass.elbow_Pos_Pick);
RobotClass.Move_Robot(r,qTraj,finger1,finger2);

%% Grab the 4th brick and place it down in new position
RobotClass.Move_Gripper(r,finger1, finger2, RobotClass.Grip_closed);
qTraj = RobotClass.Create_Trajectory(r, RobotClass.b_4_Place_Pos, RobotClass.elbow_Pos_Place);
RobotClass.Delete_Brick(b4);
RobotClass.Move_Brick(r,qTraj,finger1,finger2,RobotClass.b_4);

%% Move to 5th brick position after dropping 4th brick
RobotClass.Move_Gripper(r,finger1, finger2, RobotClass.Grip_open);
qTraj = RobotClass.Create_Trajectory(r, RobotClass.b_5_Gripper_Pos, RobotClass.elbow_Pos_Pick);
RobotClass.Move_Robot(r,qTraj,finger1,finger2);

%% Grab the 5th brick and place it down in new position
RobotClass.Move_Gripper(r,finger1, finger2, RobotClass.Grip_closed);
qTraj = RobotClass.Create_Trajectory(r, RobotClass.b_5_Place_Pos, RobotClass.elbow_Pos_Place);
RobotClass.Delete_Brick(b5);
RobotClass.Move_Brick(r,qTraj,finger1,finger2,RobotClass.b_5);

%% Move to 6th brick position after dropping 5th brick
RobotClass.Move_Gripper(r,finger1, finger2, RobotClass.Grip_open);
qTraj = RobotClass.Create_Trajectory(r, RobotClass.b_6_Gripper_Pos, RobotClass.elbow_Pos_Pick);
RobotClass.Move_Robot(r,qTraj,finger1,finger2);

%% Grab the 6th brick and place it down in new position
RobotClass.Move_Gripper(r,finger1, finger2, RobotClass.Grip_closed);
qTraj = RobotClass.Create_Trajectory(r, RobotClass.b_6_Place_Pos, RobotClass.elbow_Pos_Place);
RobotClass.Delete_Brick(b6);
RobotClass.Move_Brick(r,qTraj,finger1,finger2,RobotClass.b_6);

%% Move to 7th brick position after dropping 6th brick
RobotClass.Move_Gripper(r,finger1, finger2, RobotClass.Grip_open);
qTraj = RobotClass.Create_Trajectory(r, RobotClass.b_7_Gripper_Pos, RobotClass.elbow_Pos_Pick);
RobotClass.Move_Robot(r,qTraj,finger1,finger2);

%% Grab the 7th brick brick and place it down in new position
RobotClass.Move_Gripper(r,finger1, finger2, RobotClass.Grip_closed);
qTraj = RobotClass.Create_Trajectory(r, RobotClass.b_7_Place_Pos, RobotClass.elbow_Pos_Place);
RobotClass.Delete_Brick(b7);
RobotClass.Move_Brick(r,qTraj,finger1,finger2,RobotClass.b_7);

%% Move to 8th brick position after dropping 7th brick
RobotClass.Move_Gripper(r,finger1, finger2, RobotClass.Grip_open);
qTraj = RobotClass.Create_Trajectory(r, RobotClass.b_8_Gripper_Pos, RobotClass.elbow_Pos_Pick);
RobotClass.Move_Robot(r,qTraj,finger1,finger2);

%% Grab the 8th brick brick and place it down in new position
RobotClass.Move_Gripper(r,finger1, finger2, RobotClass.Grip_closed);
qTraj = RobotClass.Create_Trajectory(r, RobotClass.b_8_Place_Pos, RobotClass.elbow_Pos_Place);
RobotClass.Delete_Brick(b8);
RobotClass.Move_Brick(r,qTraj,finger1,finger2,RobotClass.b_8);

%% Move to 9th brick position after dropping 8th brick
RobotClass.Move_Gripper(r,finger1, finger2, RobotClass.Grip_open);
qTraj = RobotClass.Create_Trajectory(r, RobotClass.b_9_Gripper_Pos, RobotClass.elbow_Pos_Pick);
RobotClass.Move_Robot(r,qTraj,finger1,finger2);

%% Grab the the 9th brick and place it down in new position 
RobotClass.Move_Gripper(r,finger1, finger2, RobotClass.Grip_closed);
qTraj = RobotClass.Create_Trajectory(r, RobotClass.b_9_Place_Pos, RobotClass.elbow_Pos_Place);
RobotClass.Delete_Brick(b9);
RobotClass.Move_Brick(r,qTraj,finger1,finger2,RobotClass.b_9);

%% Move Robot back to zero position
RobotClass.Move_Gripper(r,finger1, finger2, RobotClass.Grip_open);
qTraj = RobotClass.Create_Trajectory(r, [0 0 0.5], RobotClass.elbow_Pos_Pick);
RobotClass.Move_Robot(r,qTraj,finger1,finger2);