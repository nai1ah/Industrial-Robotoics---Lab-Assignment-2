classdef LinearFinger < RobotBaseClass


    properties(Access = public)              
        plyFileNameStem = 'LinearFinger';
    end
    
    methods
%% Define robot Function 
function self = LinearFinger(baseTr)
			self.CreateModel();
            if nargin < 1			
				baseTr = eye(4);				
            end
            self.model.base =  self.model.base.T * baseTr  ;
            
            self.PlotAndColourRobot();         
        end

%% Create the robot model
        function CreateModel(self)   
            link(1) = Link('d',0,'a',0,'alpha',0,'qlim',deg2rad([-10 25]));   
            link(2) = Link('d',0,'a',0,'alpha',0,'qlim',deg2rad([0 0.01]));
            self.model = SerialLink(link,'name',self.name);
            self.model.plotopt = {'noshadow','noarrow','noshading','nowrist','nojaxes'};
        end
     
    end
end