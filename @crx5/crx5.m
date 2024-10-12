classdef crx5 < RobotBaseClass
    %% Linear UR3e robot based on UTS model and changed based on lecture 5 video
    % converted to a 
    % WARNING: This model has been created by UTS students in the subject
    % 41013. No guarentee is made about the accuracy or correctness of the
    % of the DH parameters of the accompanying ply files. Do not assume
    % that this matches the real robot!

    properties(Access = public)   
        plyFileNameStem = 'crx5';
    end
    
    methods
%% Constructor
function self = crx5(baseTr)
			self.CreateModel();
            if nargin < 1			
				baseTr = eye(4);				
            end
            self.model.base = self.model.base.T * baseTr * trotx(pi/2);% rotating to have robot lay flat 

            self.PlotAndColourRobot();         
        end


%% CreateModel
        function CreateModel(self)
            link(1) = Link('d',0,'a',0,'alpha',0,'qlim',[-pi pi], 'offset',0);
            link(2) = Link('d',0,'a',0,'alpha',0,'qlim',[-pi pi], 'offset',0);
            link(3) = Link('d',0,'a',0,'alpha',0,'qlim',[-pi pi], 'offset',0);
            link(4) = Link('d',0,'a',0,'alpha',0,'qlim',[-pi pi], 'offset',0);
            link(5) = Link('d',0,'a',0,'alpha',0,'qlim',[-pi pi], 'offset',0);
            link(6) = Link('d',0,'a',0,'alpha',0,'qlim',[-pi pi], 'offset',0);
             
            self.model = SerialLink(link,'name',self.name);
        end      
    end
end
