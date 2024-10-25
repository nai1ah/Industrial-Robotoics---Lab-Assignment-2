clear all
close all
clf
clc;

%% Plot the environment
Robot_Base_Class.Plot_Environment();
axis manual;
[Plate, Bottom_Bun, Cheese, Patty, Tomato, Lettuce, Top_Bun] = Robot_Base_Class.Deconstructed_Burger();
hold on

%% Pin definitions
buttonStopPin = 'D2';  % E-stop button pin
buttonResumePin = 'D3'; % Resume button pin

%% Initialize Arduino
% a = arduino('COM3','Uno');  % Connect to Arduino
% configurePin(a, buttonStopPin, 'Pullup');
% configurePin(a, buttonResumePin, 'Pullup');

%% Create E-Stop Object
% eStop = EStopUI(a, buttonStopPin, buttonResumePin);
% eStop.createUI();
% to add robots to the eStop, use eStop.addRobot()
%% Plot the CRX5 and the gripper
r1 = crx5;
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
% dobot_finger1 = LinearFinger(base_2*trotx(pi/2)); 
% dobot_finger2 = LinearFinger(base_2*troty(pi)*trotx(-pi/2));

%% Move the dobot out of the way 
qTraj = Dobot_Class.Create_Trajectory(r2,Dobot_Class.Dobot_Rest,Dobot_Class.elbow_Pos_RHS);
Dobot_Class.Move_Dobot(r2,qTraj);
qnow = r2.model.getpos;
r2.model.fkineUTS(qnow)
disp('Press enter to remove plot and continue');
 pause;
%% Move CRX to plate 
CRX5_Class.Move_Gripper(r1,crx_finger1,crx_finger2,CRX5_Class.Grip_open);
qTraj = CRX5_Class.Create_Trajectory(r1,CRX5_Class.Plate_pos_pick,CRX5_Class.elbow_Pos_Plate_Pick);
CRX5_Class.Move_crx(r1,qTraj,crx_finger1,crx_finger2);
CRX5_Class.Move_Gripper(r1,crx_finger1,crx_finger2,CRX5_Class.Grip_closed);

%% Move plate to dobot 
qTraj = CRX5_Class.Create_Trajectory(r1,CRX5_Class.Plate_pos_place,CRX5_Class.elbow_Pos_plate_Place);
Robot_Base_Class.Delete_Object(Plate);
[Plate] = CRX5_Class.Move_Plate(r1, qTraj, crx_finger1,crx_finger2,CRX5_Class.Plate_pos);
%% Move crx to rest position
CRX5_Class.Move_Gripper(r1,crx_finger1,crx_finger2,CRX5_Class.Grip_open);
qTraj = CRX5_Class.Create_Trajectory(r1,CRX5_Class.CRX_rest,CRX5_Class.elbow_Pos_Rest);
CRX5_Class.Move_crx(r1,qTraj,crx_finger1,crx_finger2);

%% Place bottom bun on plate 
qTraj = Dobot_Class.Create_Trajectory(r2,Dobot_Class.Bottombun_pos_pick,Dobot_Class.elbow_Pos_RHS);
Dobot_Class.Move_Dobot(r2,qTraj);
qTraj = Dobot_Class.Create_Trajectory(r2,Dobot_Class.Bottombun_pos_place,Dobot_Class.elbow_Pos_Place);
Robot_Base_Class.Delete_Object(Bottom_Bun);
[Bottom_Bun] = Dobot_Class.Move_Bottom_bun(r2,qTraj,Dobot_Class.Bottombun_pos);
disp('Press enter to remove plot and continue');
 % pause;
%% Place Cheese on bun 
qTraj = Dobot_Class.Create_Trajectory(r2,Dobot_Class.Cheese_pos_pick,Dobot_Class.elbow_Pos_RHS);
Dobot_Class.Move_Dobot(r2,qTraj);
qTraj = Dobot_Class.Create_Trajectory(r2,Dobot_Class.Cheese_pos_place,Dobot_Class.elbow_Pos_Place);
Robot_Base_Class.Delete_Object(Cheese);
[Cheese] = Dobot_Class.Move_Cheese(r2,qTraj,Dobot_Class.Cheese_pos);
disp('Press enter to remove plot and continue');
 % pause;
%% Place patty on bun 
qTraj = Dobot_Class.Create_Trajectory(r2,Dobot_Class.Patty_pos_pick,Dobot_Class.elbow_Pos_RHS);
Dobot_Class.Move_Dobot(r2,qTraj);
qTraj = Dobot_Class.Create_Trajectory(r2,Dobot_Class.Patty_pos_place,Dobot_Class.elbow_Pos_Place);
Robot_Base_Class.Delete_Object(Patty);
[Patty] = Dobot_Class.Move_Patty(r2,qTraj,Dobot_Class.Patty_pos);
disp('Press enter to remove plot and continue');
 % pause;
%% Place tomato on bun 
qTraj = Dobot_Class.Create_Trajectory(r2,Dobot_Class.Tomato_pos_pick,Dobot_Class.elbow_Pos_LHS);
Dobot_Class.Move_Dobot(r2,qTraj);
qTraj = Dobot_Class.Create_Trajectory(r2,Dobot_Class.Tomato_pos_place,Dobot_Class.elbow_Pos_Place);
Robot_Base_Class.Delete_Object(Tomato);
[Tomato] = Dobot_Class.Move_Tomato(r2,qTraj,Dobot_Class.Tomato_pos);
disp('Press enter to remove plot and continue');
 % pause;
%% Place lettuce on bun 
qTraj = Dobot_Class.Create_Trajectory(r2,Dobot_Class.Lettuce_pos_pick,Dobot_Class.elbow_Pos_LHS);
Dobot_Class.Move_Dobot(r2,qTraj);
qTraj = Dobot_Class.Create_Trajectory(r2,Dobot_Class.Lettuce_pos_place,Dobot_Class.elbow_Pos_Place);
Robot_Base_Class.Delete_Object(Lettuce);
[Lettuce] = Dobot_Class.Move_Lettuce(r2,qTraj,Dobot_Class.Lettuce_pos);
disp('Press enter to remove plot and continue');
 % pause;

%% Place top bun on burger
qTraj = Dobot_Class.Create_Trajectory(r2,Dobot_Class.Topbun_pos_pick,Dobot_Class.elbow_Pos_LHS);
Dobot_Class.Move_Dobot(r2,qTraj);
qTraj = Dobot_Class.Create_Trajectory(r2,Dobot_Class.Topbun_pos_place,Dobot_Class.elbow_Pos_Place);
Robot_Base_Class.Delete_Object(Top_Bun);
[Top_Bun] = Dobot_Class.Move_Top_Bun(r2,qTraj,Dobot_Class.Topbun_pos);
disp('Press enter to remove plot and continue');
 % pause;

%% Move the dobot out of the way 
qTraj = Dobot_Class.Create_Trajectory(r2,Dobot_Class.Dobot_Rest,Dobot_Class.elbow_Pos_RHS);
Dobot_Class.Move_Dobot(r2,qTraj);

%% Move crx back to plate 
qTraj = CRX5_Class.Create_Trajectory(r1,CRX5_Class.Plate_pos_place,CRX5_Class.elbow_Pos_plate_Place);
CRX5_Class.Move_crx(r1, qTraj, crx_finger1,crx_finger2);
CRX5_Class.Move_Gripper(r1,crx_finger1,crx_finger2,CRX5_Class.Grip_closed);

%% Move plate with burger back to start 
qTraj = CRX5_Class.Create_Trajectory(r1,CRX5_Class.CRX_rest,CRX5_Class.elbow_Pos_Plate_Pick);
Robot_Base_Class.Delete_burger(Plate, Bottom_Bun, Cheese, Patty, Tomato, Lettuce, Top_Bun);
[Plate, Bottom_Bun, Cheese, Patty, Tomato, Lettuce, Top_Bun] = CRX5_Class.Move_Burger(r1, qTraj, crx_finger1,crx_finger2,CRX5_Class.Plate_pos_place,Dobot_Class.Bottombun_pos_place, ...
    Dobot_Class.Cheese_pos_place,Dobot_Class.Patty_pos_place,Dobot_Class.Tomato_pos_place,Dobot_Class.Lettuce_pos_place,Dobot_Class.Topbun_pos_place);
qTraj = CRX5_Class.Create_Trajectory(r1,CRX5_Class.Plate_pos_pick,CRX5_Class.elbow_Pos_Plate_Pick);
Robot_Base_Class.Delete_burger(Plate, Bottom_Bun, Cheese, Patty, Tomato, Lettuce, Top_Bun);
[Plate, Bottom_Bun, Cheese, Patty, Tomato, Lettuce, Top_Bun] = CRX5_Class.Move_Burger(r1, qTraj, crx_finger1,crx_finger2,CRX5_Class.Plate_pos_place,Dobot_Class.Bottombun_pos_place, ...
    Dobot_Class.Cheese_pos_place,Dobot_Class.Patty_pos_place,Dobot_Class.Tomato_pos_place,Dobot_Class.Lettuce_pos_place,Dobot_Class.Topbun_pos_place);
CRX5_Class.Move_Gripper(r1,crx_finger1,crx_finger2,CRX5_Class.Grip_open);
qTraj = CRX5_Class.Create_Trajectory(r1,CRX5_Class.CRX_rest,CRX5_Class.elbow_Pos_Rest);
CRX5_Class.Move_crx(r1,qTraj,crx_finger1,crx_finger2);