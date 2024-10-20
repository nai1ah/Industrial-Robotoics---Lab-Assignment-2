classdef CRX5_Class < handle

    properties (Constant)
%% Declaring guesses for joint positions 
elbow_Pos_Rest = [-0.0628 0.9425 -1.5708 -2.4504 0] % elbow guess for rest position      
elbow_Pos_Plate_Pick = [1.2566 0.6912 -1.2566 -2.5761 0]; %picking up the plate
elbow_Pos_plate_Place = [-0.8796 0.6283 -1.1310 -2.6389 0]; %placing the plate
%% Declaring gripper positions 
Grip_open = deg2rad([25 0]);
Grip_closed = deg2rad([-10 0]);
%% Declaring positions for items
Plate_pos = [0,1.4,0.55];
Plate_pos_build = [0,0.1,0.55];
Bottombun_pos = [0.25,-0.42,0.55];
Cheese_pos = [0.25,-0.3,0.55];
Patty_pos = [0.25,-0.15,0.55];
tomato_pos = [-0.25,-0.42,0.55];
lettuce_pos = [-0.25,-0.3,0.56];
Topbun_pos = [-0.25,-0.15,0.55];

%% Declaring positions for items
Plate_pos_pick = [0,1.34,0.65];
Plate_pos_place = [0,0.16,0.65];
CRX_rest = [0, 0.4,0.7]
% Bottombun_pos = [0.25,-0.42,0.55];
% Cheese_pos = [0.25,-0.3,0.55];
% Patty_pos = [0.25,-0.15,0.55];
% tomato_pos = [-0.25,-0.42,0.55];
% lettuce_pos = [-0.25,-0.3,0.56];
% Topbun_pos = [-0.25,-0.15,0.55];

    end

    methods (Static) 
%% Function to generate a set of q values based on  jtraj - quintic polynomial
        function qtrajec = Create_Trajectory(robot,Position,jointGuess)
            steps = 100;
            qNow = robot.model.getpos();
            T = transl(Position)*trotx(pi)*troty(0)*trotz(pi/2);    
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
        function Move_Plate(self,qArray,finger1, finger2,plate)
        %Read the ply file in faces, vertices, and color it
            [f,v,data] = plyread('plate.ply');%,'tri'
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            plateVertexCount = size(v,1);
            PlateMesh_h = trisurf(f,v(:,1)+plate(1,1),v(:,2)+plate(1,2), v(:,3)+plate(1,3) ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','none','EdgeLighting','none');
        % for loop to move the plate, arm and gripper all together 
            for i=1:size(qArray,1)
        % Moving plate along with the gripper and arm
                plate = self.model.fkineUTS(qArray(i,:));
                Plate_Pose = plate*transl(0,-0.06,0.05) ; %% translate the position of the brick to move with gripper
                UpdatedPoints = (Plate_Pose * [v,ones(plateVertexCount,1)]')'; % updated position of brick and apply to each vertex 
                PlateMesh_h.Vertices = UpdatedPoints(:,1:3);%The vertices are columns 1 to 3 and all rows of UpdatedPoints
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
            
            hold on;
            %Plot the surrounding wall
            surf([2,2;2,2],[-2,-2;4,4],[0.01,4;0.01,4],'CData',imread('Environment.jpg'),'FaceColor','texturemap');
            surf([-2,-2;2,2],[4,4;4,4],[0.01,4;0.01,4],'CData',imread('Environment.jpg'),'FaceColor','texturemap');
        end

        %% function for creating burger parts 
        function [Plate, Bottom_Bun, Cheese, Patty, Tomato, Lettuce, Top_Bun] = Deconstructed_Burger() 
            Plate = PlaceObject('plate.ply',CRX5_Class.Plate_pos);
            Bottom_Bun = PlaceObject('bottombun.ply',CRX5_Class.Bottombun_pos);
            Cheese = PlaceObject('cheese.ply',CRX5_Class.Cheese_pos);
            Patty = PlaceObject('patty.ply',CRX5_Class.Patty_pos);
            Tomato = PlaceObject('tomato.ply',CRX5_Class.tomato_pos);
            Lettuce = PlaceObject('lettuce.ply',CRX5_Class.lettuce_pos);
            Top_Bun = PlaceObject('topbun.ply',CRX5_Class.Topbun_pos);
        end
        %% function to delete each brick safely through a catch function 
        function Delete_Object(object)
            try delete(object);
            catch ME
            end
        end
 
    end
end

