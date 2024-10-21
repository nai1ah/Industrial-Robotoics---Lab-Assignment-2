clear all
close all
clf
clc;

%% Plot the environment
CRX5_Class.Plot_Environment()
axis manual;
[Plate, Bottom_Bun, Cheese, Patty, Tomato, Lettuce, Top_Bun] = CRX5_Class.Deconstructed_Burger();
hold on

%% Plot the CRX5 and the gripper
% Call the LinearUR3e
r1 = crx5;
% r1.model.base = trotz(pi); % rotate the crx robot about the z axis 
r1.model.base.t = [-0.35,0.75,0.55]; % moving the base of the robot  
q_1_Now = r1.model.getpos();
r1.model.animate(q_1_Now); % plotting the new orientation of the robot
hold on 

% Take the base of end-effector 
base_1 = r1.model.fkineUTS(q_1_Now);

%Call the gripper with 2 fingers and plot the calculated base 
crx_finger1 = LinearFinger(base_1*trotx(pi/2)); 
crx_finger2 = LinearFinger(base_1*troty(pi)*trotx(-pi/2));

%% Plot the dobot magician and the gripper
% Call the LinearUR3e
r2 = DobotMagician;
r2.model.base = trotz(pi/2);
r2.model.base.t = [0,-0.25,0.55];
q_2_Now = r2.model.getpos();

r2.model.animate(q_2_Now);
hold on 

% Take the base of end-effector 
base_2 = r2.model.fkineUTS(q_2_Now);

%Call the gripper with 2 fingers and plot the calculated base 
dobot_finger1 = LinearFinger(base_2*trotx(pi/2)); 
dobot_finger2 = LinearFinger(base_2*troty(pi)*trotx(-pi/2));

%% Move CRX to plate 
CRX5_Class.Move_Gripper(r1,crx_finger1,crx_finger2,CRX5_Class.Grip_open);
qTraj = CRX5_Class.Create_Trajectory(r1,CRX5_Class.Plate_pos_pick,CRX5_Class.elbow_Pos_Plate_Pick);
CRX5_Class.Move_crx(r1,qTraj,crx_finger1,crx_finger2);
CRX5_Class.Move_Gripper(r1,crx_finger1,crx_finger2,CRX5_Class.Grip_closed);

%% Move plate to dobot 
qTraj = CRX5_Class.Create_Trajectory(r1,CRX5_Class.Plate_pos_place,CRX5_Class.elbow_Pos_plate_Place);
CRX5_Class.Delete_Object(Plate);
CRX5_Class.Move_Plate(r1, qTraj, crx_finger1,crx_finger2,CRX5_Class.Plate_pos);
%% Move crx to rest position
CRX5_Class.Move_Gripper(r1,crx_finger1,crx_finger2,CRX5_Class.Grip_open);
qTraj = CRX5_Class.Create_Trajectory(r1,CRX5_Class.CRX_rest,CRX5_Class.elbow_Pos_Rest);
CRX5_Class.Move_crx(r1,qTraj,crx_finger1,crx_finger2);

%% Place bottom bun on plate 


%% Place Cheese on bun 


%% Place patty on bun 


%% Place tomato on bun 


%% Place lettuce on bun 


%% Place top bun on burger


%% Move crx back to plate 
qTraj = CRX5_Class.Create_Trajectory(r1,CRX5_Class.Plate_pos_place,CRX5_Class.elbow_Pos_plate_Place);
CRX5_Class.Move_crx(r1, qTraj, crx_finger1,crx_finger2);
CRX5_Class.Move_Gripper(r1,crx_finger1,crx_finger2,CRX5_Class.Grip_closed);

%% Move plate with burger back to start 
qTraj = CRX5_Class.Create_Trajectory(r1,CRX5_Class.Plate_pos_pick,CRX5_Class.elbow_Pos_Plate_Pick);
CRX5_Class.Delete_Object(Plate); 
CRX5_Class.Move_Plate(r1, qTraj, crx_finger1,crx_finger2,CRX5_Class.Plate_pos_place);
CRX5_Class.Move_Gripper(r1,crx_finger1,crx_finger2,CRX5_Class.Grip_open);
qTraj = CRX5_Class.Create_Trajectory(r1,CRX5_Class.CRX_rest,CRX5_Class.elbow_Pos_Rest);
CRX5_Class.Move_crx(r1,qTraj,crx_finger1,crx_finger2);