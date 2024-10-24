classdef Robot_Base_Class < handle

    properties (Constant)
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
Bottombun_pos_pick = [0.25,-0.42,0.65];
Cheese_pos_pick = [0.25,-0.3,0.65];
Patty_pos_pick = [0.25,-0.15,0.65];
tomato_pos_pick = [-0.25,-0.42,0.65];
lettuce_pos_pick = [-0.25,-0.3,0.66];
Topbun_pos_pick = [-0.25,-0.15,0.65];

%% Declaring positions for placing burger parts 
Bottombun_pos_place = [0,0.1,0.65];
Cheese_pos_place = [0,0.1,0.67];
Patty_pos_place = [0,0.1,0.68];
tomato_pos_place = [0,0.1,0.70];
lettuce_pos_place = [0,0.1,0.71];
Topbun_pos_place = [0,0.1,0.72];

    end

    methods (Static) 
%% Function to move the gripper to either close or open 
        function Move_Gripper(r,finger1, finger2, gripper_positon)
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
            Plate = PlaceObject('plate.ply',Robot_Base_Class.Plate_pos);
            Bottom_Bun = PlaceObject('bottombun.ply',Robot_Base_Class.Bottombun_pos);
            Cheese = PlaceObject('cheese.ply',Robot_Base_Class.Cheese_pos);
            Patty = PlaceObject('patty.ply',Robot_Base_Class.Patty_pos);
            Tomato = PlaceObject('tomato.ply',Robot_Base_Class.tomato_pos);
            Lettuce = PlaceObject('lettuce.ply',Robot_Base_Class.lettuce_pos);
            Top_Bun = PlaceObject('topbun.ply',Robot_Base_Class.Topbun_pos);
        end
        %% function to delete each brick safely through a catch function 
        function Delete_Object(object)
            try delete(object);
            catch ME
            end
        end
 
    end
end