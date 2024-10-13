classdef crx5 < RobotBaseClass


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
            self.model.base = self.model.base.T * baseTr * transl(0,0,0);% rotating to have robot lay flat 

            self.PlotAndColourRobot();         
        end


%% CreateModel
        function CreateModel(self)
            % link(1) = Link('d',0,'a',0,'alpha',pi/2,'qlim',[-pi pi], 'offset',0);
            link(1) = Link('d',0.25,'a',0,'alpha',pi/2,'qlim', [-pi pi], 'offset',0);
            link(2) = Link('d',0.07,'a',0.405,'alpha',0,'qlim', [-pi pi], 'offset', 0);
            link(3) = Link('d',-0.01,'a',0.42,'alpha',0,'qlim',[-pi pi],'offset', 0);
            link(4) = Link('d',0.09,'a',0,'alpha',-pi/2,'qlim',[-pi pi], 'offset',0);
            link(5) = Link('d',	0.155,'a',0,'alpha',0,'qlim',[-pi pi], 'offset', 0);
            
            self.model = SerialLink(link,'name',self.name);
            self.model.plotopt = {'noshadow','noarrow','noshading','nowrist','nojaxes'};
        end 

    end
end
