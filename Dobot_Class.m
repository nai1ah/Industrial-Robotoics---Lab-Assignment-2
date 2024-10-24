classdef Dobot_Class < handle

    properties (Constant)
%% Declaring guesses for joint positions 
elbow_Pos_Place = [0 0.9774 1.4765 0 0] % elbow guess for rest position      
elbow_Pos_RHS = [-1.6022 0.9774 1.4765 0 0]; %picking up the plate
elbow_Pos_LHS = [1.6493 0.9774 1.4765 0 0]; %placing the plate
%% Declaring gripper positions 
Grip_open = deg2rad([25 0]);
Grip_closed = deg2rad([-5 0]);
%% Declaring positions for items
Plate_pos = [0,1.4,0.55];
Plate_pos_build = [0,0.1,0.55];
Bottombun_pos = [0.25,-0.42,0.55];
Cheese_pos = [0.25,-0.3,0.55];
Patty_pos = [0.25,-0.15,0.55];
tomato_pos = [-0.25,-0.42,0.55];
lettuce_pos = [-0.25,-0.3,0.56];
Topbun_pos = [-0.25,-0.15,0.55];

%% Declaring positions for grabbing burger parts 
Bottombun_pos_pick = [0.25,-0.42,0.6];
Cheese_pos_pick = [0.25,-0.3,0.6];
Patty_pos_pick = [0.25,-0.15,0.6];
tomato_pos_pick = [-0.25,-0.42,0.6];
lettuce_pos_pick = [-0.25,-0.3,0.6];
Topbun_pos_pick = [-0.25,-0.15,0.6];

%% Declaring positions for placing burger parts 
Bottombun_pos_place = [0,0.1,0.65];
Cheese_pos_place = [0,0.1,0.67];
Patty_pos_place = [0,0.1,0.68];
tomato_pos_place = [0,0.1,0.70];
lettuce_pos_place = [0,0.1,0.71];
Topbun_pos_place = [0,0.1,0.72];

    end

    methods (Static) 
%% Function to generate a set of q values based on  jtraj - quintic polynomial
        function qtrajec = Create_Trajectory(robot,Position,jointGuess)
            steps = 100;
            qNow = robot.model.getpos();
            T = transl(Position)*trotx(0)*troty(0)*trotz(0);    
            qMove = wrapToPi(robot.model.ikcon(T,jointGuess));
            qtrajec = jtraj(qNow,qMove,steps);
        end

        %% Function to move arm and gripper to passed trajectory
        function  Move_Dobot(r,qtrajec)

            for i = 1:size(qtrajec,1)
                q = qtrajec(i,:);
                r.model.animate(q);
                pause(0.0005)
            end
        end
        %% Function to move the arm, gripper and grabbed brick to the desired position 
        function Move_Bottom_bun(self,qArray,Bottom_bun)
        %Read the ply file in faces, vertices, and color it
            [f,v,data] = plyread('bottombun.ply','tri');%
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            plateVertexCount = size(v,1);
            PlateMesh_h = trisurf(f,v(:,1)+Bottom_bun(1,1),v(:,2)+Bottom_bun(1,2), v(:,3)+Bottom_bun(1,3) ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','none','EdgeLighting','none');
        % for loop to move the plate, arm and gripper all together 
            for i=1:size(qArray,1)
        % Moving plate along with the gripper and arm
                Bottom_bun = self.model.fkineUTS(qArray(i,:));
                Plate_Pose = Bottom_bun*transl(0,0,-0.07) ; %% translate the position of the brick to move with gripper
                UpdatedPoints = (Plate_Pose * [v,ones(plateVertexCount,1)]')'; % updated position of brick and apply to each vertex 
                PlateMesh_h.Vertices = UpdatedPoints(:,1:3);%The vertices are columns 1 to 3 and all rows of UpdatedPoints
        % Move the robot arm through the trajectory      
                self.model.animate(qArray(i,:));
                pause(0.0005);
            end
        end
 
    end
end
