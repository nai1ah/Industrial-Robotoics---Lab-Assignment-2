classdef CRX5_Class < handle

    properties (Constant)
%% Declaring guesses for joint positions 
        elbow_Pos_Plate_Pick = [ 1.1868    0.5585   -1.6755   -0.4887    0.4189]; %picking up the plate
        elbow_Pos_plate_Place = [   -0.9076    0.5585   -1.6755   -0.4887    0.4189]; %placing the plate
%% Declaring gripper positions 
        Grip_open = deg2rad([25 0]);
        Grip_closed = deg2rad([11 0]);
%% Declaring positions for items
Plate_pos = [0,1.4,0.55];
Bottombun_pos = [0.25,-0.42,0.55];
Cheese_pos = [0.25,-0.3,0.55];
Patty_pos = [0.25,-0.15,0.55];
tomato_pos = [-0.25,-0.42,0.55];
lettuce_pos = [-0.25,-0.3,0.56];
Topbun_pos = [-0.25,-0.15,0.55];

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
        function  Move_crx(r,qtrajec,crx_finger1,crx_finger2)

            for i = 1:size(qtrajec,1)
                q = qtrajec(i,:);
                base = r.model.fkineUTS(q);
                r.model.animate(q);
                q_f1 = crx_finger1.model.getpos(); % get current pos of robot
                q_f2 = crx_finger2.model.getpos();
                crx_finger1.model.base = base * trotx(pi/2);
                crx_finger1.model.animate(q_f1);
                crx_finger2.model.base = base * troty(pi) * trotx(-pi/2);
                crx_finger2.model.animate(q_f2);
                pause(0.0005)
            end
        end

%% Function to move the gripper to either close or open 
        function Move_Gripper(r, crx_finger1, crx_finger2, gripper_positon)
            steps = 100;
            q_f1 = crx_finger1.model.getpos();
            q_f2 = crx_finger2.model.getpos();
            q_f1_traj = jtraj(q_f1,gripper_positon,steps);
            q_f2_traj = jtraj(q_f2,gripper_positon,steps);
            qNow = r.model.getpos();
            base = r.model.fkineUTS(qNow);
            for i = 1:size(q_f1_traj,1) % moves each finger individually to sim moving together
                crx_finger1.model.base = base * trotx(pi/2);
                crx_finger1.model.animate(q_f1_traj(i, :));
                crx_finger2.model.base = base * troty(pi) * trotx(-pi/2);
                crx_finger2.model.animate(q_f2_traj(i, :));
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
%% Function to plot the environment and its safety features
        function Plot_Environment()
            fig = figure(1); % Ensure figure 1 is used
            set(fig, 'Name', 'Robot Simulation Environment', 'NumberTitle', 'off');
            axis([-2 1.5 -1.5 1.5 -0.01 2]); % Set axis limits
        % Place fences 1:4
            PlaceObject('BarrierThick.ply',[1,2.5,0]);
            hold on;
            PlaceObject('BarrierThick.ply',[-1,2.5,0]);
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

            fence_9 =PlaceObject('BarrierThick.ply',[-1.5,-1.3,0]);
            verts = [get(fence_9,'Vertices'), ones(size(get(fence_9,'Vertices'),1),1)] * trotz(pi/2);
            set(fence_9,'Vertices',verts(:,1:3))
            hold on;

            fence_10 =PlaceObject('BarrierThick.ply',[-1.5,1.3,0]);
            verts = [get(fence_10,'Vertices'), ones(size(get(fence_10,'Vertices'),1),1)] * trotz(pi/2);
            set(fence_10,'Vertices',verts(:,1:3))
            hold on;

        %Plot the concrete ground 
            set(0,'DefaultFigureWindowStyle','docked');
            surf([-2,-2;2,2] ...
                ,[-2,4;-2,4] ...
                ,[0.01,0.01;0.01,0.01] ...
                ,'CData',imread('Concrete.jpg') ...
                ,'FaceColor','texturemap');
        % Place extra safety features being a table and the estop on top 
            PlaceObject('Table.ply',[1,3,0]);
            PlaceObject('Table.ply',[-0.4,-0.23,0]);
            PlaceObject('Table.ply',[-0.4,0.5,0]);
            PlaceObject('Table.ply',[0.4,-0.23,0]);
            PlaceObject('Table.ply',[0.4,0.5,0]);
            PlaceObject('Table.ply',[-0.4,1.23,0]);
            PlaceObject('Table.ply',[0.4,1.23,0]);
            PlaceObject('emergencyStopButton.ply',[1,3,0.55]);
            PlaceObject('plate.ply',CRX5_Class.Plate_pos);
            PlaceObject('bottombun.ply',CRX5_Class.Bottombun_pos);
            PlaceObject('cheese.ply',CRX5_Class.Cheese_pos);
            PlaceObject('patty.ply',CRX5_Class.Patty_pos);
            PlaceObject('tomato.ply',CRX5_Class.tomato_pos);
            PlaceObject('lettuce.ply',CRX5_Class.lettuce_pos);
            PlaceObject('topbun.ply',CRX5_Class.Topbun_pos);
            hold on;
            %Plot the surrounding wall
            surf([-2,-2;-2,-2],[-2,-2;4,4],[0.01,4;0.01,4],'CData',imread('Environment.jpg'),'FaceColor','texturemap');
            surf([-2,-2;2,2],[4,4;4,4],[0.01,4;0.01,4],'CData',imread('Environment.jpg'),'FaceColor','texturemap');
        end
 
    end
end

