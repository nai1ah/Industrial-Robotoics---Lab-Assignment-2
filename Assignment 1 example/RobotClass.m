classdef RobotClass < handle

    properties (Constant)
%% declaring initial position of bricks
        b_1 = [0.3 0 0]; % brick 1
        b_2 = [0.3 0.1765 0];% brick 2
        b_3 = [0.3 0.353 0];% brick 3
        b_4 = [0.417 0 0];% brick 4
        b_5 = [0.417 0.1765 0];% brick 5
        b_6 = [0.417 0.353 0];% brick 6
        b_7 = [0.534 0 0];% brick 7
        b_8 = [0.534 0.1765 0];% brick 8
        b_9 = [0.534 0.353 0];% brick 9
%% Declaring Gripper position above brick (+0.15 in z direction)
        b_1_Gripper_Pos = [0.3 0 0.09]; % gripper position above brick 1
        b_2_Gripper_Pos = [0.3 0.1765 0.09]; % gripper position above brick 2
        b_3_Gripper_Pos = [0.3 0.353 0.09]; % gripper position above brick 3
        b_4_Gripper_Pos = [0.417 0 0.09]; % gripper position above brick 4
        b_5_Gripper_Pos = [0.417 0.1765 0.09]; % gripper position above brick 5
        b_6_Gripper_Pos = [0.417 0.353 0.09]; % gripper position above brick 6
        b_7_Gripper_Pos = [0.534 0 0.09]; % gripper position above brick 7
        b_8_Gripper_Pos = [0.534 0.1765 0.09]; % gripper position above brick 8
        b_9_Gripper_Pos = [0.534 0.353 0.09]; % gripper position above brick 9
%% Declaring final positions for bricks (-0.35 in x, seperated by 0.1465 in y, seperated by 0.0335 in z)  
        b_1_Place_Pos = [-0.35 0 0.09]; % gripper position to place brick 1
        b_2_Place_Pos = [-0.35 0.1465 0.09]; % gripper position to place brick 2
        b_3_Place_Pos = [-0.35 0.293 0.09]; % gripper position to place brick 3
        b_4_Place_Pos = [-0.35 0 0.1235]; % gripper position to place brick 4
        b_5_Place_Pos = [-0.35 0.1465 0.1235]; % gripper position to place brick 5
        b_6_Place_Pos = [-0.35 0.293 0.1235]; % gripper position to place brick 6
        b_7_Place_Pos = [-0.35 0 0.1570]; % gripper position to place brick 7
        b_8_Place_Pos = [-0.35 0.1465 0.1570]; % gripper position to place brick 8 
        b_9_Place_Pos = [-0.35 0.293 0.1570]; % gripper position to place brick 9
%% Declaring guesses for joint positions 
        elbow_Pos_Pick = [-pi/6 pi -pi -pi/3 -pi/3 pi/2 -pi/2]; %picking up the brick
        elbow_Pos_Place = [-pi/6 pi -pi/3 pi/2 -2*pi/3 -pi/2 pi/2]; %placing the brick
%% Declaring gripper positions 
        Grip_open = deg2rad([25 0]);
        Grip_closed = deg2rad([11 0]);
    end

    methods (Static) 
%% Function to generate a set of q values based on  jtraj - quintic polynomial
        function qtrajec = Create_Trajectory(robot,brickPosition,jointGuess)
            steps = 100;
            qNow = robot.model.getpos();
            T = transl(brickPosition)*trotx(pi)*troty(0)*trotz(0);    
            qMove = wrapToPi(robot.model.ikcon(T,jointGuess));
            qtrajec = jtraj(qNow,qMove,steps);
        end
%% Function to move arm and gripper to passed trajectory
        function  Move_Robot(r,qtrajec,finger1,finger2)

            for i = 1:size(qtrajec,1)
                q = qtrajec(i,:);
                base = r.model.fkineUTS(q);
                r.model.animate(q);
                q_f1 = finger1.model.getpos(); % get current pos of robot
                q_f2 = finger2.model.getpos();
                finger1.model.base = base * trotx(pi/2);
                finger1.model.animate(q_f1);
                finger2.model.base = base * troty(pi) * trotx(-pi/2);
                finger2.model.animate(q_f2);
                pause(0.0005)
            end
        end

%% Function to move the gripper to either close or open 
        function Move_Gripper(r, finger1, finger2, gripper_positon)
            steps = 100;
            q_f1 = finger1.model.getpos();
            q_f2 = finger2.model.getpos();
            q_f1_traj = jtraj(q_f1,gripper_positon,steps);
            q_f2_traj = jtraj(q_f2,gripper_positon,steps);
            qNow = r.model.getpos();
            base = r.model.fkineUTS(qNow);
            for i = 1:size(q_f1_traj,1) % moves each finger individually to sim moving together
                finger1.model.base = base * trotx(pi/2);
                finger1.model.animate(q_f1_traj(i, :));
                finger2.model.base = base * troty(pi) * trotx(-pi/2);
                finger2.model.animate(q_f2_traj(i, :));
                pause(0.0005)
            end
            
        end
%% Function to move the arm, gripper and grabbed brick to the desired position 
        function Move_Brick(self,qArray,finger1, finger2,bricknum)
        %Create a log file using the log4matlab tool from matlab 
            logFileName = strcat('logLinearUR3',self.model.name,'.log');
            L = log4matlab(logFileName);
            disp(strcat("Logging on file: ",logFileName));
            num = 1;
        %Read the ply file in faces, vertices, and color it
            [f,v,data] = plyread('HalfSizedRedGreenBrick.ply','tri');
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            BrickVertexCount = size(v,1);
            BrickMesh_h = trisurf(f,v(:,1)+bricknum(1,1),v(:,2)+bricknum(1,2), v(:,3)+bricknum(1,3) ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','none','EdgeLighting','none');
        % for loop to move the brick, arm and gripper all together 
            for i=1:size(qArray,1)
        % Moving brick along with the gripper and arm
                brick = self.model.fkineUTS(qArray(i,:));
                BrickPose = brick*transl(0,0,0.05) ; %% translate the position of the brick to move with gripper
                UpdatedPoints = (BrickPose * [v,ones(BrickVertexCount,1)]')'; % updated position of brick and apply to each vertex 
                BrickMesh_h.Vertices = UpdatedPoints(:,1:3);%The vertices are columns 1 to 3 and all rows of UpdatedPoints
        % Move the robot arm through the trajectory      
                self.model.animate(qArray(i,:));  
        % Move the gripper along with the arm by updating the base and
                base = self.model.fkineUTS(qArray(i,:));
                q_f1 = finger1.model.getpos();
                q_f2 = finger2.model.getpos();
                finger1.model.base = base * trotx(pi/2);
                finger1.model.animate(q_f1);
                finger2.model.base = base * troty(pi) * trotx(-pi/2);
                finger2.model.animate(q_f2);
                pause(0.0005);
        %Display the current position of the end-effector every 'freq' times
                freq = 5;
                if mod(num,freq) == 0
                    currPosition = transl(self.model.fkine(qArray(i,:)).T)'; % using fkine to determine current ee pos
                    message = strcat('DEBUG: ',' Current position = ',num2str(currPosition));
                    L.mlog = {L.DEBUG,'LinearUR3',message};
                    disp(message);
                end
                num = num + 1;
            end
        end
%% Function to plot and calculate the volume based on tutorial 3 for spray gun 
        function [Volume_Plot] = Plot_Volume(self)
            stepRads = deg2rad(60);
            stepmeter = 0.2;
            qlim = self.model.qlim;
            X_axis = [4 13 4 6 13 13];
            pointCloudeSize = prod(X_axis);
            pointCloud = zeros(pointCloudeSize,3);
            counter = 1;
            for q1 = qlim(1,1):stepmeter:qlim(1,2)
                for q2 = qlim(2,1):stepRads:qlim(2,2)
                    for q3 = qlim(3,1):stepRads:qlim(3,2)
                        for q4 = qlim(4,1):stepRads:qlim(4,2)
                            for q5 = qlim(5,1):stepRads:qlim(5,2)
                                for q6 = qlim(6,1):stepRads:qlim(6,2)
                                    q7 = 0;
                                    q = [q1,q2,q3,q4,q5,q6,q7];
                                    tr = self.model.fkineUTS(q);
                                    pointCloud(counter,:) = tr(1:3,4)';
                                    counter = counter + 1;
                                end
                            end
                        end
                    end
                end
            end
        %Plot the volume of the robot arm based on the Qlim defined
            hold on;
            figure(1);
            Volume_Plot = plot3(pointCloud(:,1),pointCloud(:,2),pointCloud(:,3),'o','Color','r');
            drawnow();
            hold on;
        % Calculate the max spread for each axis
            X_axis = (max(pointCloud(:,1)) - min(pointCloud(:,1)))/2;
            Y_axis = (max(pointCloud(:,2)) - min(pointCloud(:,2)))/2;
            Z_axis = (max(pointCloud(:,3)) - min(pointCloud(:,3)))/2;
        % Calculate its volume  based on it beaing an elipsoid
            volume = (4/3)*pi*X_axis*Y_axis*Z_axis;
            disp(strcat("Approximate volume (m^3) = ",num2str(volume)));
        end

%% Function to plot the environment and its safety features
        function Plot_Environment()
            fig = figure(1); % Ensure figure 1 is used
            set(fig, 'Name', 'Robot Simulation Environment', 'NumberTitle', 'off');
            axis([-2 1.5 -1.5 1.5 -0.01 2]); % Set axis limits
        % Place fences 1:4
            PlaceObject('BarrierThick.ply',[1.2,1.2,0]);
            hold on;
            PlaceObject('BarrierThick.ply',[-1.2,1.2,0]);
            hold on;
            PlaceObject('BarrierThick.ply',[0.5,-1,0]);
            hold on;
            PlaceObject('BarrierThick.ply',[-0.5,-1,0]);
            hold on;
        %Plot fence 5:8 and rotate them into position
            fence_5 = PlaceObject('BarrierThick.ply',[-0.5,1.3,0]);
            verts = [get(fence_5,'Vertices'), ones(size(get(fence_5,'Vertices'),1),1)] * trotz(pi/2);
            set(fence_5,'Vertices',verts(:,1:3))
            hold on;

            fence_6 =PlaceObject('BarrierThick.ply',[0.5,1.3,0]);
            verts = [get(fence_6,'Vertices'), ones(size(get(fence_6,'Vertices'),1),1)] * trotz(pi/2);
            set(fence_6,'Vertices',verts(:,1:3))
            hold on;

            fence_7 =PlaceObject('BarrierThick.ply',[-0.5,-1.3,0]);
            verts = [get(fence_7,'Vertices'), ones(size(get(fence_7,'Vertices'),1),1)] * trotz(pi/2);
            set(fence_7,'Vertices',verts(:,1:3))
            hold on;

            fence_8 =PlaceObject('BarrierThick.ply',[0.5,-1.3,0]);
            verts = [get(fence_8,'Vertices'), ones(size(get(fence_8,'Vertices'),1),1)] * trotz(pi/2);
            set(fence_8,'Vertices',verts(:,1:3))
            hold on;

        %Plot the concrete ground 
            set(0,'DefaultFigureWindowStyle','docked');
            surf([-2,-2;2,2] ...
                ,[-2,4;-2,4] ...
                ,[0.01,0.01;0.01,0.01] ...
                ,'CData',imread('Concrete.jpg') ...
                ,'FaceColor','texturemap');
        % Place extra safety features being a table and the estop on top 
            PlaceObject('Table.ply',[1,2.5,0]);
            PlaceObject('emergencyStopButton.ply',[1,2.3,0.55]);
        end

%% Function to plot the bricks
        function [b1, b2, b3, b4, b5, b6, b7, b8, b9] = Plot_Brick()
        % placing bricks 1:9
            b1 = PlaceObject('HalfSizedRedGreenBrick.ply' ,RobotClass.b_1); 
            b2 = PlaceObject('HalfSizedRedGreenBrick.ply' ,RobotClass.b_2);
            b3 = PlaceObject('HalfSizedRedGreenBrick.ply' ,RobotClass.b_3);
            b4 = PlaceObject('HalfSizedRedGreenBrick.ply' ,RobotClass.b_4);
            b5 = PlaceObject('HalfSizedRedGreenBrick.ply' ,RobotClass.b_5);
            b6 = PlaceObject('HalfSizedRedGreenBrick.ply' ,RobotClass.b_6);
            b7 = PlaceObject('HalfSizedRedGreenBrick.ply' ,RobotClass.b_7);
            b8 = PlaceObject('HalfSizedRedGreenBrick.ply' ,RobotClass.b_8);
            b9 = PlaceObject('HalfSizedRedGreenBrick.ply' ,RobotClass.b_9);
        end
%% function to delete each brick safely through a catch function 
        function Delete_Brick(brick)
            try delete(brick);
            catch ME
            end
        end
 
    end
end