classdef Dobot_Class < handle

    properties (Constant)
%% Declaring guesses for joint positions 
elbow_Pos_Place = [0 0.9774 1.4765 0 0] % elbow guess for rest position      
elbow_Pos_RHS = [-pi/2 pi/3 pi/2 0 0]; %picking up ingredients on RHS
elbow_Pos_LHS = [pi/2 pi/3 pi/2 0 0]; %placing up ingredients on LHS
%% Declaring gripper positions 
Grip_open = deg2rad([25 0]);
Grip_closed = deg2rad([-5 0]);
%% Declaring positions for items
Plate_pos = [0,1.4,0.55];
Plate_pos_build = [0,0.1,0.55];
Bottombun_pos = [0.25,-0.42,0.55];
Cheese_pos = [0.25,-0.3,0.55];
Patty_pos = [0.25,-0.15,0.55];
Tomato_pos = [-0.25,-0.42,0.55];
Lettuce_pos = [-0.25,-0.3,0.56];
Topbun_pos = [-0.25,-0.15,0.55];

%% Declaring positions for grabbing burger parts 
Dobot_Rest = [0.25,-0.3,0.8];
Bottombun_pos_pick = [0.25,-0.42,0.61];
Cheese_pos_pick = [0.25,-0.3,0.6];
Patty_pos_pick = [0.25,-0.15,0.61];
Tomato_pos_pick = [-0.25,-0.42,0.61];
Lettuce_pos_pick = [-0.25,-0.3,0.61];
Topbun_pos_pick = [-0.25,-0.15,0.61];

%% Declaring positions for placing burger parts 
Bottombun_pos_place = [0,0.1,0.63];
Cheese_pos_place = [0,0.1,0.64];
Patty_pos_place = [0,0.1,0.645];
Tomato_pos_place = [0,0.1,0.66];
Lettuce_pos_place = [0,0.1,0.675];
Topbun_pos_place = [0,0.1,0.682];

    end

    methods (Static) 
%% Function to generate a set of q values based on  jtraj - quintic polynomial
        function qtrajec = Create_Trajectory(robot,Position,jointGuess)
            steps = 150;
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
        %% Function to move Bottom bun
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

        %% Function to Move cheese  
        function Move_Cheese(self,qArray,cheese)
        %Read the ply file in faces, vertices, and color it
            [f,v,data] = plyread('cheese.ply','tri');%
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            plateVertexCount = size(v,1);
            PlateMesh_h = trisurf(f,v(:,1)+cheese(1,1),v(:,2)+cheese(1,2), v(:,3)+cheese(1,3) ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','none','EdgeLighting','none');
        % for loop to move the plate, arm and gripper all together 
            for i=1:size(qArray,1)
        % Moving plate along with the gripper and arm
                cheese = self.model.fkineUTS(qArray(i,:));
                Plate_Pose = cheese*transl(0,0,-0.06) ; %% translate the position of the brick to move with gripper
                UpdatedPoints = (Plate_Pose * [v,ones(plateVertexCount,1)]')'; % updated position of brick and apply to each vertex 
                PlateMesh_h.Vertices = UpdatedPoints(:,1:3);%The vertices are columns 1 to 3 and all rows of UpdatedPoints
        % Move the robot arm through the trajectory      
                self.model.animate(qArray(i,:));
                pause(0.0005);
            end
        end

        %% Function to Move patty  
        function Move_Patty(self,qArray,patty)
        %Read the ply file in faces, vertices, and color it
            [f3,v3,data3] = plyread('patty.ply','tri');%
            patty_Colours = [data3.vertex.red, data3.vertex.green, data3.vertex.blue] / 255;
            patty_VertexCount = size(v3,1);
            patty_Mesh_h = trisurf(f3,v3(:,1)+patty(1,1),v3(:,2)+patty(1,2), v3(:,3)+patty(1,3) ...
                ,'FaceVertexCData',patty_Colours,'EdgeColor','none','EdgeLighting','none');
        % for loop to move the plate, arm and gripper all together 
            for i=1:size(qArray,1)
        % Moving plate along with the gripper and arm
                patty = self.model.fkineUTS(qArray(i,:));
                Patty_Pose = patty*transl(0,0,-0.07) ; %% translate the position of the brick to move with gripper
                UpdatedPoints3 = (Patty_Pose * [v3,ones(patty_VertexCount,1)]')'; % updated position of brick and apply to each vertex 
                patty_Mesh_h.Vertices = UpdatedPoints3(:,1:3);%The vertices are columns 1 to 3 and all rows of UpdatedPoints
        % Move the robot arm through the trajectory      
                self.model.animate(qArray(i,:));
                pause(0.0005);
            end
        end

%% Function to Move tomato  
        function Move_Tomato(self,qArray,tomato)
        %Read the ply file for tomato
            [f4,v4,data4] = plyread('tomato.ply','tri');%
            Tomato_vertexColours = [data4.vertex.red, data4.vertex.green, data4.vertex.blue] / 255;
            Tomato_VertexCount = size(v4,1);
            Tomato_Mesh_h = trisurf(f4,v4(:,1)+tomato(1,1),v4(:,2)+tomato(1,2), v4(:,3)+tomato(1,3) ...
                ,'FaceVertexCData',Tomato_vertexColours,'EdgeColor','none','EdgeLighting','none');
        % for loop to move the tomato, arm and gripper all together 
            for i=1:size(qArray,1)
        % Moving tomato along with the gripper and arm
                tomato = self.model.fkineUTS(qArray(i,:));
                Tomato_Pose = tomato*transl(0,0,-0.06) ; %% translate the position of the brick to move with gripper
                UpdatedPoints4 = (Tomato_Pose * [v4,ones(Tomato_VertexCount,1)]')'; % updated position of brick and apply to each vertex 
                Tomato_Mesh_h.Vertices = UpdatedPoints4(:,1:3);%The vertices are columns 1 to 3 and all rows of UpdatedPoints
        % Move the robot arm through the trajectory      
                self.model.animate(qArray(i,:));
                pause(0.0005);
            end
        end
        
%% Function to Move lettuce  
        function Move_Lettuce(self,qArray,lettuce)
        %Read the ply file for lettuce
            [f5,v5,data5] = plyread('lettuce.ply','tri');%
            Lettuce_vertexColours = [data5.vertex.red, data5.vertex.green, data5.vertex.blue] / 255;
            Lettuce_VertexCount = size(v5,1);
            Lettuce_Mesh_h = trisurf(f5,v5(:,1)+lettuce(1,1),v5(:,2)+lettuce(1,2), v5(:,3)+lettuce(1,3) ...
                ,'FaceVertexCData',Lettuce_vertexColours,'EdgeColor','none','EdgeLighting','none');
        % for loop to move the plate, arm and gripper all together 
            for i=1:size(qArray,1)
        % Moving lettuce along with the gripper and arm
                lettuce = self.model.fkineUTS(qArray(i,:));
                Lettuce_Pose = lettuce*transl(0,0,-0.06) ; %% translate the position of the brick to move with gripper
                UpdatedPoints5 = (Lettuce_Pose * [v5,ones(Lettuce_VertexCount,1)]')'; % updated position of brick and apply to each vertex 
                Lettuce_Mesh_h.Vertices = UpdatedPoints5(:,1:3);%The vertices are columns 1 to 3 and all rows of UpdatedPoints
        % Move the robot arm through the trajectory      
                self.model.animate(qArray(i,:));
                pause(0.0005);
            end
        end
%% Function to Move top bun  
function Move_Top_Bun(self,qArray,top_bun)
        %Read the ply file for top bun
            [f6,v6,data6] = plyread('topbun.ply','tri');%
            TopBun_vertexColours = [data6.vertex.red, data6.vertex.green, data6.vertex.blue] / 255;
            TopBun_VertexCount = size(v6,1);
            TopBun_Mesh_h = trisurf(f6,v6(:,1)+top_bun(1,1),v6(:,2)+top_bun(1,2), v6(:,3)+top_bun(1,3) ...
                ,'FaceVertexCData',TopBun_vertexColours,'EdgeColor','none','EdgeLighting','none');
        % for loop to move the top bun, arm and gripper all together 
            for i=1:size(qArray,1)
        % Moving top bun along with the gripper and arm
                top_bun = self.model.fkineUTS(qArray(i,:));
                TopBun_Pose = top_bun*transl(0,0,-0.07) ; %% translate the position of the brick to move with gripper
                UpdatedPoints6 = (TopBun_Pose * [v6,ones(TopBun_VertexCount,1)]')'; % updated position of brick and apply to each vertex 
                TopBun_Mesh_h.Vertices = UpdatedPoints6(:,1:3);%The vertices are columns 1 to 3 and all rows of UpdatedPoints
        % Move the robot arm through the trajectory      
                self.model.animate(qArray(i,:));
                pause(0.0005);
            end
        end
    end
end

