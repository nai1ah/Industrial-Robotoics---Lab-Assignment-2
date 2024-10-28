classdef Dobot_Class < handle

    properties (Constant)
%% Declaring guesses for joint positions 
elbow_Pos_Place = [0 pi/3 pi/2 0 0] % elbow guess for rest position      
elbow_Pos_RHS = [-pi/4 pi/3 pi/2 0 -pi/6]; %picking up ingredients on RHS
elbow_Pos_LHS = [pi/2 pi/3 pi/2 0 0]; %placing up ingredients on LHS
%% Declaring gripper positions 
Grip_open = deg2rad([25 0]);
Grip_closed = deg2rad([-5 0]);
%% Declaring positions for items
Plate_pos = [0,1.4,0.55];
Plate_pos_build = [0,0.1,0.55];
% RHS
Bottombun_pos = [0.3,-0.42,0.55];
Cheese_pos = [0.3,-0.3,0.55];
Patty_pos = [0.3,-0.15,0.55];
% LHS
Tomato_pos = [-0.25,-0.42,0.55];
Lettuce_pos = [-0.25,-0.3,0.56];
Topbun_pos = [-0.25,-0.15,0.55];

%% Declaring positions for grabbing burger parts 
Dobot_Rest = [0.25,-0.3,0.8];
% RHS 
Bottombun_pos_pick = [0.3,-0.42,0.61];
Cheese_pos_pick = [0.3,-0.3,0.6];
Patty_pos_pick = [0.3,-0.15,0.61];
% LHS
Tomato_pos_pick = [-0.25,-0.42,0.61];
Lettuce_pos_pick = [-0.25,-0.3,0.61];
Topbun_pos_pick = [-0.25,-0.15,0.61];

%% Declaring positions for placing burger parts 
Bottombun_pos_place = [0,0.08,0.63];
Cheese_pos_place = [0,0.08,0.64];
Patty_pos_place = [0,0.08,0.645];
Tomato_pos_place = [0,0.08,0.66];
Lettuce_pos_place = [0,0.08,0.675];
Topbun_pos_place = [0,0.08,0.682];

    end

    methods (Static) 
%% Function to generate a set of q values based on  jtraj - quintic polynomial
        function qtrajec = Create_Trajectory(robot, Position, jointGuess, face, vertex, faceNormals)
            % if no obstacle, continue
            if nargin < 5
                steps = 100;
                qNow = robot.model.getpos();
                T = transl(Position)*trotx(0)*troty(0)*trotz(0);    
                qMove = wrapToPi(robot.model.ikcon(T,jointGuess));
                qtrajec = jtraj(qNow,qMove,steps);
                return;
            end            
            qNow = robot.model.getpos();
            T = transl(Position) * trotx(pi) * troty(0) * trotz(pi/2); 
            qEnd = wrapToPi(robot.model.ikcon(T, jointGuess)); 
            
            % Generate collision-free trajectory
            qtrajec = Dobot_Class.collisionAvoidance(robot,qNow, qEnd, face, vertex, faceNormals);
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
        function [Bottom_Bun] = Move_Bottom_bun(self,qArray,Bottom_bun)
        %Read the ply file in faces, vertices, and color it
            [f,v,data] = plyread('bottombun.ply','tri');%
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            plateVertexCount = size(v,1);
            Bottom_bun_Mesh_h = trisurf(f,v(:,1)+Bottom_bun(1,1),v(:,2)+Bottom_bun(1,2), v(:,3)+Bottom_bun(1,3) ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','none','EdgeLighting','none');
        % for loop to move the plate, arm and gripper all together 
            for i=1:size(qArray,1)
        % Moving plate along with the gripper and arm
                Bottom_bun = self.model.fkineUTS(qArray(i,:));
                Plate_Pose = Bottom_bun*transl(0,0,-0.07) ; %% translate the position of the brick to move with gripper
                UpdatedPoints = (Plate_Pose * [v,ones(plateVertexCount,1)]')'; % updated position of brick and apply to each vertex 
                Bottom_bun_Mesh_h.Vertices = UpdatedPoints(:,1:3);%The vertices are columns 1 to 3 and all rows of UpdatedPoints
        % Move the robot arm through the trajectory      
                self.model.animate(qArray(i,:));
                pause(0.0005);
            end
            Bottom_Bun = Bottom_bun_Mesh_h;
        end

        %% Function to Move cheese  
        function [Cheese] = Move_Cheese(self,qArray,cheese)
        %Read the ply file in faces, vertices, and color it
            [f,v,data] = plyread('cheese.ply','tri');%
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            plateVertexCount = size(v,1);
            Cheese_Mesh_h = trisurf(f,v(:,1)+cheese(1,1),v(:,2)+cheese(1,2), v(:,3)+cheese(1,3) ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','none','EdgeLighting','none');
        % for loop to move the plate, arm and gripper all together 
            for i=1:size(qArray,1)
        % Moving plate along with the gripper and arm
                cheese = self.model.fkineUTS(qArray(i,:));
                Plate_Pose = cheese*transl(0,0,-0.06) ; %% translate the position of the brick to move with gripper
                UpdatedPoints = (Plate_Pose * [v,ones(plateVertexCount,1)]')'; % updated position of brick and apply to each vertex 
                Cheese_Mesh_h.Vertices = UpdatedPoints(:,1:3);%The vertices are columns 1 to 3 and all rows of UpdatedPoints
        % Move the robot arm through the trajectory      
                self.model.animate(qArray(i,:));
                pause(0.0005);
            end
            Cheese = Cheese_Mesh_h;
        end

        %% Function to Move patty  
        function [Patty] = Move_Patty(self,qArray,patty)
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
            Patty = patty_Mesh_h;
        end

%% Function to Move tomato  
        function [Tomato] = Move_Tomato(self,qArray,tomato)
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
            Tomato = Tomato_Mesh_h;
        end
        
%% Function to Move lettuce  
        function [Lettuce] = Move_Lettuce(self,qArray,lettuce)
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
            Lettuce = Lettuce_Mesh_h;
        end
%% Function to Move top bun  
function [Top_Bun] = Move_Top_Bun(self,qArray,top_bun)
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
            Top_Bun = TopBun_Mesh_h;
        end
        %% Collision Avoidance
function qMatrix = collisionAvoidance(robot, qStart, qEnd, face, vertex, faceNormals)
    qWaypoints = [qStart; qEnd]; 
    isCollision = true; 
    checkedWaypoint = 1;  % Track up to where the path has been checked
    qMatrix = []; % Initialise q matrix
    maxAttempts = 100;  % Maximum number of waypoint attempts
    attemptCount = 0; 

    while (isCollision)
        startWaypoint = checkedWaypoint;  % Start checking from last successful waypoint
        for i = startWaypoint:size(qWaypoints, 1) - 1
            % Interpolate between waypoints
            qMatrixStart = Dobot_Class.InterpolateWaypointRadians(qWaypoints(i:i+1,:), deg2rad(10));
            
            % Check if the interpolated path is collision-free
            if ~Dobot_Class.IsCollision(robot, qMatrixStart, face, vertex, faceNormals)
                % No collision, proceed with movement
                qMatrix = [qMatrix; qMatrixStart]; %#ok<AGROW>
                isCollision = false;
                checkedWaypoint = i + 1;
                
                % Try joining to the final goal
                qMatrixStart = Dobot_Class.InterpolateWaypointRadians([qMatrix(end,:); qEnd], deg2rad(10));
                if ~Dobot_Class.IsCollision(robot, qMatrixStart, face, vertex, faceNormals)
                    qMatrix = [qMatrix; qMatrixStart];
                    disp('Reached goal without collision.');
                    isCollision = false;
                    return;  % Exit as the goal is reached without collision
                end
            else
                % Collision detected, pick a random waypoint that is collision-free
                disp('Collision detected. Avoiding...');
                qAvoid = Dobot_Class.getRandomWaypoint(robot);
                while Dobot_Class.IsCollision(robot,qAvoid,face,vertex,faceNormals)
                    qAvoid = Dobot_Class.getRandomWaypoint(robot);
                end
                attemptCount = attemptCount + 1;  % Increment the attempt counter
                
                % If maximum attempts reached, abandon the path planning
                if attemptCount >= maxAttempts
                    disp('Abandoning path planning: too many collision attempts.');
                    qMatrix = [];  % Return an empty qMatrix to signal failure
                    return;
                end

                qWaypoints = [qWaypoints(1:i,:); qAvoid; qWaypoints(i+1:end,:)];
                isCollision = true;
                break;  % Restart the loop with the new random waypoint
            end
        end
    end
end

%% Find a random set of waypoints
function qAvoid = getRandomWaypoint(robot)
    links = length(robot.model.links);
    qAvoid = (2 * rand(1, links) - 1) * pi; 

    % Calculate the end-effector pose
    transforms = Dobot_Class.GetLinkPoses(qAvoid, robot); 
    endEffectorZ = transforms(3, 4, end); % Z position of the end effector

    % Adjust the joint angles if the end-effector is below the robot base
    while (endEffectorZ < robot.model.base.t(3))
        qAvoid = (2 * rand(1, links) - 1) * pi; 

        % Find waypoint above robot base to avoid collision with ground
        transforms = Dobot_Class.GetLinkPoses(qAvoid, robot); 
        endEffectorZ = transforms(3, 4, end); % Z position of the end effector
        if (endEffectorZ > robot.model.base.t(3))
            return;
        end
    end
end

%% FineInterpolation
        function qMatrix = FineInterpolation(q1,q2,maxStepRadians)
            if nargin < 3
                maxStepRadians = deg2rad(10);
            end
                
            steps = 100;
            while ~isempty(find(maxStepRadians < abs(diff(jtraj(q1,q2,steps))),1))
                steps = steps + 1;
            end
            qMatrix = jtraj(q1,q2,steps);
        end

%% InterpolateWaypointRadians
% Given a set of waypoints, finely intepolate them
        function qMatrix = InterpolateWaypointRadians(waypointRadians,maxStepRadians)
            if nargin < 2
                maxStepRadians = deg2rad(1);
            end
            
            qMatrix = [];
            for i = 1: size(waypointRadians,1)-1
                qMatrix = [qMatrix ; Dobot_Class.FineInterpolation(waypointRadians(i,:),waypointRadians(i+1,:),maxStepRadians)]; %#ok<AGROW>
            end
        end
%% Check if there is collision
        function result = IsCollision(robot,qMatrix,faces,vertex,faceNormals,returnOnceFound)
            if nargin < 6
                returnOnceFound = true;
            end
            result = false;
            
            for qIndex = 1:size(qMatrix,1)
                % Get the transform of every joint (i.e. start and end of every link)
                tr = Dobot_Class.GetLinkPoses(qMatrix(qIndex,:), robot);
            
                % Go through each link and also each triangle face
                for i = 1 : size(tr,3)-1    
                    for faceIndex = 1:size(faces,1)
                        vertOnPlane = vertex(faces(faceIndex,1)',:);
                        [intersectP,check] = LinePlaneIntersection(faceNormals(faceIndex,:),vertOnPlane,tr(1:3,4,i)',tr(1:3,4,i+1)'); 
                        if check == 1 && Dobot_Class.IsIntersectionPointInsideTriangle(intersectP,vertex(faces(faceIndex,:)',:))
                            plot3(intersectP(1),intersectP(2),intersectP(3),'g*');
                            disp('Intersection');
                            result = true;
                            if returnOnceFound
                                return
                            end
                        end
                    end    
                end
            end
        end

        %% IsIntersectionPointInsideTriangle
function result = IsIntersectionPointInsideTriangle(intersectP,triangleVerts)

u = triangleVerts(2,:) - triangleVerts(1,:);
v = triangleVerts(3,:) - triangleVerts(1,:);

uu = dot(u,u);
uv = dot(u,v);
vv = dot(v,v);

w = intersectP - triangleVerts(1,:);
wu = dot(w,u);
wv = dot(w,v);

D = uv * uv - uu * vv;

% Get and test parametric coords (s and t)
s = (uv * wv - vv * wu) / D;
if (s < 0.0 || s > 1.0)        % intersectP is outside Triangle
    result = 0;
    return;
end

t = (uv * wu - uu * wv) / D;
if (t < 0.0 || (s + t) > 1.0)  % intersectP is outside Triangle
    result = 0;
    return;
end

result = 1;                      % intersectP is in Triangle
end
%% Find poses
        function [ transforms ] = GetLinkPoses( q, robot)
            links = robot.model.links;
            transforms = zeros(4, 4, length(links) + 1);
            transforms(:,:,1) = robot.model.base;
            
            for i = 1:length(links)
                L = links(1,i);
                
                current_transform = transforms(:,:, i);
                
                current_transform = current_transform * trotz(q(1,i) + L.offset) * ...
                transl(0,0, L.d) * transl(L.a,0,0) * trotx(L.alpha);
                transforms(:,:,i + 1) = current_transform;
            end
        end
    end
end

