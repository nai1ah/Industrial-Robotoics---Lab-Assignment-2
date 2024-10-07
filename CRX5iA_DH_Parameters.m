clc;
clear all;

L1 = Link('d',0.185,'a',0,'alpha',pi/2,'offset',0,'qlim',[deg2rad(-200),deg2rad(200)]);
L2 = Link('d',0.298,'a',-0.410,'alpha',0,'offset',0,'qlim',[deg2rad(-180),deg2rad(180)]);
L3 = Link('d',-0.298,'a',0,'alpha',pi/2,'offset',0,'qlim',[deg2rad(-317.5),deg2rad(317.5)]);
L4 = Link('d',-0.430,'a',0,'alpha',-pi/2,'offset',0,'qlim',[deg2rad(-190),deg2rad(190)]);
L5 = Link('d',0.130,'a',0,'alpha',-pi/2,'offset',0,'qlim',[deg2rad(-180),deg2rad(180)]);
L6 = Link('d',0.145,'a',0,'alpha',0,'offset',0,'qlim',[deg2rad(-225),deg2rad(225)]);

robot = SerialLink([L1 L2 L3 L4 L5 L6],'name','FANUC CRX-5iA');

q = zeros(1,6);       
robot.plot(q);       
robot.teach(); 
