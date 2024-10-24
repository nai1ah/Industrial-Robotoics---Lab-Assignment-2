classdef CRX5_Class < handle

    properties (Constant)
%% Declaring guesses for joint positions 
elbow_Pos_Rest = [0 pi/3 -pi -3*pi/4 0] % elbow guess for rest position      
elbow_Pos_Plate_Pick = [5*pi/12 pi/4 -5*pi/12 -5*pi/6 0]; %picking up the plate
elbow_Pos_plate_Place = [-pi/4 pi/4 -pi/3 -5*pi/6 0]; %placing the plate

%% Declaring gripper positions 
Grip_open = deg2rad([25 0]);
Grip_closed = deg2rad([-5 0]);

%% Declaring positions for items
Plate_pos = [0,1.4,0.55];
Plate_pos_build = [-0.005,0.13,0.55];

%% Declaring positions for items
Plate_pos_pick = [0,1.46,0.65];
Plate_pos_place = [-0.005,0.13,0.65];
CRX_rest = [0, 0.4,0.7];

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
        function  Move_crx(r,qtrajec,finger1,finger2)

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
            [f,v,data] = plyread('plate.ply','tri');%
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            plateVertexCount = size(v,1);
            PlateMesh_h = trisurf(f,v(:,1)+plate(1,1),v(:,2)+plate(1,2), v(:,3)+plate(1,3) ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','none','EdgeLighting','none');
        % for loop to move the plate, arm and gripper all together 
            for i=1:size(qArray,1)
        % Moving plate along with the gripper and arm
                plate = self.model.fkineUTS(qArray(i,:));
                Plate_Pose = plate*transl(0.06,0,0.1)*trotx(pi)*troty(0)*trotz(pi/2); %% translate the position of the brick to move with gripper
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
%% Function to move the arm, gripper and grabbed brick to the desired position 
function Move_Burger(self,qArray,finger1, finger2,plate,Bottom_bun,cheese,patty,tomato,lettuce,top_bun)
        %Read the ply file in faces, vertices, and color it
            [f,v,data] = plyread('plate.ply','tri');%
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            plateVertexCount = size(v,1);
            PlateMesh_h = trisurf(f,v(:,1)+plate(1,1),v(:,2)+plate(1,2), v(:,3)+plate(1,3) ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','none','EdgeLighting','none');
        % Read ply file for bottom bun 
            [f1,v1,data1] = plyread('bottombun.ply','tri');%
            vertexColours1 = [data1.vertex.red, data1.vertex.green, data1.vertex.blue] / 255;
            Bottom_Bun_VertexCount = size(v1,1);
            Bottom_Bun_Mesh_h = trisurf(f1,v1(:,1)+Bottom_bun(1,1),v1(:,2)+Bottom_bun(1,2), v1(:,3)+Bottom_bun(1,3) ...
                ,'FaceVertexCData',vertexColours1,'EdgeColor','none','EdgeLighting','none');
        %Read the ply file for cheese 
            [f2,v2,data2] = plyread('cheese.ply','tri');%
            cheese_Colours = [data2.vertex.red, data2.vertex.green, data2.vertex.blue] / 255;
            Cheese_VertexCount = size(v2,1);
            Cheese_Mesh_h = trisurf(f2,v2(:,1)+cheese(1,1),v2(:,2)+cheese(1,2), v2(:,3)+cheese(1,3) ...
                ,'FaceVertexCData',cheese_Colours,'EdgeColor','none','EdgeLighting','none');
         %Read the ply file for patty
            [f3,v3,data3] = plyread('patty.ply','tri');%
            patty_Colours = [data3.vertex.red, data3.vertex.green, data3.vertex.blue] / 255;
            patty_VertexCount = size(v3,1);
            patty_Mesh_h = trisurf(f3,v3(:,1)+patty(1,1),v3(:,2)+patty(1,2), v3(:,3)+patty(1,3) ...
                ,'FaceVertexCData',patty_Colours,'EdgeColor','none','EdgeLighting','none');
        %Read the ply file for tomato
            [f4,v4,data4] = plyread('tomato.ply','tri');%
            Tomato_vertexColours = [data4.vertex.red, data4.vertex.green, data4.vertex.blue] / 255;
            Tomato_VertexCount = size(v4,1);
            Tomato_Mesh_h = trisurf(f4,v4(:,1)+tomato(1,1),v4(:,2)+tomato(1,2), v4(:,3)+tomato(1,3) ...
                ,'FaceVertexCData',Tomato_vertexColours,'EdgeColor','none','EdgeLighting','none');
        %Read the ply file for lettuce
            [f5,v5,data5] = plyread('lettuce.ply','tri');%
            Lettuce_vertexColours = [data5.vertex.red, data5.vertex.green, data5.vertex.blue] / 255;
            Lettuce_VertexCount = size(v5,1);
            Lettuce_Mesh_h = trisurf(f5,v5(:,1)+lettuce(1,1),v5(:,2)+lettuce(1,2), v5(:,3)+lettuce(1,3) ...
                ,'FaceVertexCData',Lettuce_vertexColours,'EdgeColor','none','EdgeLighting','none');
        %Read the ply file for top bun
            [f6,v6,data6] = plyread('topbun.ply','tri');%
            TopBun_vertexColours = [data6.vertex.red, data6.vertex.green, data6.vertex.blue] / 255;
            TopBun_VertexCount = size(v6,1);
            TopBun_Mesh_h = trisurf(f6,v6(:,1)+top_bun(1,1),v6(:,2)+top_bun(1,2), v6(:,3)+top_bun(1,3) ...
                ,'FaceVertexCData',TopBun_vertexColours,'EdgeColor','none','EdgeLighting','none');

        % for loop to move the plate, arm and gripper all together 
            for i=1:size(qArray,1)
        % Moving plate along with the gripper and arm
                plate = self.model.fkineUTS(qArray(i,:));
                Plate_Pose = plate*transl(0.06,0,0.1)*trotx(pi)*troty(0)*trotz(pi/2); %% translate the position of the brick to move with gripper
                UpdatedPoints = (Plate_Pose * [v,ones(plateVertexCount,1)]')'; % updated position of brick and apply to each vertex 
                PlateMesh_h.Vertices = UpdatedPoints(:,1:3);%The vertices are columns 1 to 3 and all rows of UpdatedPoints
        % Moving Bottom bun along with the gripper and arm
                Bottom_bun = self.model.fkineUTS(qArray(i,:));
                Bottom_bun_Pose = Bottom_bun*transl(0.06,0,0.09)*trotx(pi)*troty(0)*trotz(pi/2); %% translate the position of the brick to move with gripper
                UpdatedPoints1 = (Bottom_bun_Pose * [v1,ones(Bottom_Bun_VertexCount,1)]')'; % updated position of brick and apply to each vertex 
                Bottom_Bun_Mesh_h.Vertices = UpdatedPoints1(:,1:3);
        % Moving cheese along with the gripper and arm
                cheese = self.model.fkineUTS(qArray(i,:));
                cheese_Pose = cheese*transl(0.06,0,0.07)*trotx(pi)*troty(0)*trotz(pi/2); %% translate the position of the brick to move with gripper
                UpdatedPoints2 = (cheese_Pose * [v2,ones(Cheese_VertexCount,1)]')'; % updated position of brick and apply to each vertex 
                Cheese_Mesh_h.Vertices = UpdatedPoints2(:,1:3);
        % Moving patty along with the gripper and arm
                patty = self.model.fkineUTS(qArray(i,:));
                Patty_Pose = patty*transl(0.06,0,0.065)*trotx(pi)*troty(0)*trotz(pi/2); %% translate the position of the brick to move with gripper
                UpdatedPoints3 = (Patty_Pose * [v3,ones(patty_VertexCount,1)]')'; % updated position of brick and apply to each vertex 
                patty_Mesh_h.Vertices = UpdatedPoints3(:,1:3);
        % Moving tomato along with the gripper and arm
                tomato = self.model.fkineUTS(qArray(i,:));
                Tomato_Pose = tomato*transl(0.06,0,0.045)*trotx(pi)*troty(0)*trotz(pi/2); %% translate the position of the brick to move with gripper
                UpdatedPoints4 = (Tomato_Pose * [v4,ones(Tomato_VertexCount,1)]')'; % updated position of brick and apply to each vertex 
                Tomato_Mesh_h.Vertices = UpdatedPoints4(:,1:3);%
        % Moving lettuce along with the gripper and arm
                lettuce = self.model.fkineUTS(qArray(i,:));
                Lettuce_Pose = lettuce*transl(0.06,0,0.035)*trotx(pi)*troty(0)*trotz(pi/2); %% translate the position of the brick to move with gripper
                UpdatedPoints5 = (Lettuce_Pose * [v5,ones(Lettuce_VertexCount,1)]')'; % updated position of brick and apply to each vertex 
                Lettuce_Mesh_h.Vertices = UpdatedPoints5(:,1:3);
        % Moving top bun along with the gripper and arm
                top_bun = self.model.fkineUTS(qArray(i,:));
                TopBun_Pose = top_bun*transl(0.06,0,0.029)*trotx(pi)*troty(0)*trotz(pi/2); %% translate the position of the brick to move with gripper
                UpdatedPoints6 = (TopBun_Pose * [v6,ones(TopBun_VertexCount,1)]')'; % updated position of brick and apply to each vertex 
                TopBun_Mesh_h.Vertices = UpdatedPoints6(:,1:3);

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
    end
end

