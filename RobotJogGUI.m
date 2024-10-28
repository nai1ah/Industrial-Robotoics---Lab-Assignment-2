classdef RobotJogGUI < handle
    properties
        robot             % The robot model
        waypoints         % Array to store waypoints
        jointAngleTexts   % Text objects to display joint angles
        HF                % Handle for figure
        joy               % Joystick object
        dof               % Degrees of freedom of the robot
        q                 % Current joint angles
    end
    
    methods
        function obj = RobotJogGUI(robot)
            obj.robot = robot;
            obj.dof = length(robot.model.links);
            obj.q = zeros(1, obj.dof);
            obj.waypoints = [];

            % Create the GUI
            obj.createGUI();
            
            % Setup joystick
            id = 1;  % Adjust as needed
            obj.joy = vrjoystick(id);
            caps(obj.joy);  % Display joystick information
            
            % Start the simulation
            obj.runSimulation();
        end
        
        function createGUI(obj)
            % Create the main figure
            obj.HF = figure('Name', 'Jogging Control','Position',[100,100,600,400] ...
                ,'WindowStyle', 'normal');
            
            % Create text displays for individual joint angles
            obj.jointAngleTexts = gobjects(1, obj.dof);
            for i = 1:obj.dof
                obj.jointAngleTexts(i) = uicontrol('Style', 'text', 'Position', ...
                    [50, 350 - (i - 1) * 30, 200, 30], 'String', ...
                    sprintf('Joint %d Angle: 0.00', i), 'FontSize', 12);
            end
            
            % Button to save the current waypoint
            uicontrol('Style', 'pushbutton', 'Position', [400, 50, 150, 30], ...
                'String', 'Save Waypoint', 'Callback', @(~, ~) obj.saveWaypoint());
            
            % Button to display saved waypoints
            uicontrol('Style', 'pushbutton', 'Position', [400, 90, 150, 30], ...
                'String', 'Display Waypoints', 'Callback', @(~, ~) obj.displayWaypoints());

            % Exit button
            uicontrol('Style', 'pushbutton', 'Position', [400, 10, 150, 30], ...
                'String', 'Exit', 'Callback', @(~,~) obj.exitGUI());
        end

         function exitGUI(obj)
            close(obj.HF);  % Close the GUI window
            disp('Exiting GUI and stopping simulation.');
        end
        
        function runSimulation(obj)

            % Initialize joint angles
            obj.robot.model.delay = 0.001;
            duration = 300;
            dt = 0.15;
            n = 0;
            tic;            
            while (toc < duration) && isvalid(obj.HF)
                n = n + 1;
                
                % Read joystick inputs and calculate joint velocity
                [axes, buttons, ~] = read(obj.joy);
                dq = obj.calculateJointVelocity(axes, buttons);
                
                % Update joint angles and keep within limits
                obj.q = obj.q + dq' * dt;

                % Update text fields for joint angles
                for i = 1:obj.dof
                    set(obj.jointAngleTexts(i), 'String', sprintf('Joint %d Angle: %.2f', ...
                        i, obj.q(i) * (180/pi)));
                end
                
                % Animate the robot with the new joint angles
                obj.robot.model.animate(obj.q);
                
                % Wait until loop time elapsed
                if toc > dt * n
                    warning('Loop %i took too much time - consider increasing dt', n);
                end
                while toc < dt * n
                end
            end
        end
        
        function dq = calculateJointVelocity(obj, axes, buttons)
            % Calculate end-effector velocity based on joystick inputs
            % Velocity commands and joint update
            Kv = 0.3; % linear velocity gain
            Kw = 0.8; % angular velocity gain
            vx = Kv * axes(1);
            vy = Kv * axes(2);
            vz = Kv * (buttons(5) - buttons(7));
            wx = Kw * axes(4);
            wy = Kw * axes(3);
            wz = Kw * (buttons(6) - buttons(8));
            
            dx = [vx; vy; vz; wx; wy; wz];
            
            % Calculate joint velocities
            lambda = 0.5;
            J = obj.robot.model.jacob0(obj.q);
            Jinv_dls = inv(J' * J + lambda^2 * eye(obj.dof)) * J';
            dq = Jinv_dls * dx;
        end
        
        function saveWaypoint(obj)
            obj.waypoints = [obj.waypoints; obj.q];  % Save current joint angles to waypoints
            disp('Waypoint saved:');
            disp(obj.q)
        end
        
        function displayWaypoints(obj)
            if isempty(obj.waypoints)
                disp('No waypoints saved.');
            else
                disp('Saved Waypoints:');
                disp(obj.waypoints);
                
                % Display waypoints in a new figure
                figure('Name', 'Saved Waypoints', 'Position', [750, 100, 300, 300]);
                uitable('Data', obj.waypoints, 'ColumnName', strcat('Joint ', string(1:obj.dof)), ...
                    'Position', [10, 10, 280, 280]);
            end
        end
    end
end
