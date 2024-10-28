classdef EStopUI < handle
    properties
        robots % Array of robot objects
        eStopStatus % Overall E-stop status
        resumeStatus % Resume status
        arduino % Arduino object
        buttonStopPin % Pin for E-stop button
        buttonResumePin % Pin for Resume button
    end
    
    methods
        %% Constructor
        function obj = EStopUI(arduino, buttonStopPin, buttonResumePin)
            obj.arduino = arduino;
            obj.buttonStopPin = buttonStopPin;
            obj.buttonResumePin = buttonResumePin;
            obj.eStopStatus = false; % Initialize E-stop as not activated
            obj.resumeStatus = true;
            obj.robots = []; % Initialize empty robot array
        end
        %% Add Robots
        function addRobot(obj, robot)
            % Add a robot to the manager
            obj.robots{end+1} = robot;
        end
        %% Create UI
        function createUI(obj)
            % Create UI
            fig = uifigure('Name', 'E-Stop Control', 'Position', [500 500 300 100]);
            
            % E-stop button
            btnStop = uibutton(fig, 'push', ...
                'Text', 'Activate E-Stop', ...
                'Position', [100, 40, 100, 30], ...
                'ButtonPushedFcn', @(btnStop, event) obj.toggleEStop(btnStop));
            
            % Resume button
            btnResume = uibutton(fig, 'push', ...
                'Text', 'Resume', ...
                'Position', [100, 10, 100, 30], ...
                'ButtonPushedFcn', @(btnResume, event) obj.resumeSystem());
        end
        %% Activate E-Stop
        function activateEStop(obj)
            disp('Emergency Stop Activated');
            obj.eStopStatus = true;
            obj.resumeStatus = false;
            % Notify all robots to stop
            while obj.resumeStatus == false
                obj.checkButtons();
                % Loop through robots to freeze
                for i = 1:length(obj.robots)
                    robot = obj.robots{i};  % Access the robot using curly braces
                    qStop = robot.model.getpos();  % Call getpos method on the model
                    robot.model.animate(qStop);     % Animate the robot's model
                end
            end
        end
        %% Deactivate E-Stop
        function deactivateEStop(obj)
            disp('Emergency Stop Disengaged');
            obj.eStopStatus = false; % Change E-Stop status to false     
        end
        %% Resume System
        function resumeSystem(obj)
            if obj.eStopStatus == false
                obj.resumeStatus = true;
                disp('System Resumed');
            else
                disp('Cannot Resume: E-Stop still engaged');
            end
        end
        %% Check Buttons
        function checkButtons(obj)
            stopButtonState = readDigitalPin(obj.arduino, obj.buttonStopPin);
            resumeButtonState = readDigitalPin(obj.arduino, obj.buttonResumePin);
            if stopButtonState ==1 && obj.eStopStatus == false
                obj.activateEStop();
            end
            % If stop button pressed, exit
            if stopButtonState == 1 && obj.eStopStatus == true
                obj.deactivateEStop();
            end
            % if resume button pressed, call function
            if resumeButtonState == 1
                obj.resumeSystem();
            end
            pause(0.1);
        end
        %% Toggle E-Stop
        function toggleEStop(obj, btnStop)
            if obj.eStopStatus == true
                btnStop.Text = 'Activate E-Stop';  % Change button text to 'Activate E-Stop'
                obj.deactivateEStop();  % Call to deactivate
            else
                btnStop.Text = 'Deactivate E-Stop';  % Change button text to 'Deactivate E-Stop'
                obj.activateEStop();  % Call to activate
            end
        end
    end
end
