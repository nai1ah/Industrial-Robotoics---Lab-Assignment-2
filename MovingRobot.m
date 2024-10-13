clear all
clf

% RobotClass.Plot_Environment()
axis equal;
hold on 
r = crx5;

% Take the base of end-effector 
qNow = r.model.getpos();
base = r.model.fkineUTS(qNow)

%Call the gripper with 2 fingers and plot the calculated base 
finger1 = LinearFinger(base*trotx(pi/2)); 
finger2 = LinearFinger(base*troty(pi)*trotx(-pi/2));
%% 1. Give an end-effector pose. Determine a joint state

t = [0.3 0 0];


%jointState = r.model.ikcon(T1)

%% 2. Move the robot to required joint state
qtrajec = CRX5_Class.Create_Trajectory(r, t, CRX5_Class.elbow_Pos_Pick);
CRX5_Class.Move_Robot(r,qtrajec,finger1,finger2);

%% Provide evidence that the joint states satisfy the given pose within +-5mm

qNow = r.model.getpos();
r.model.teach(qNow);
