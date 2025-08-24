function animation = drone_gate_Animation(x,y,z,roll,pitch,yaw,gate,playspeed, bRecordVideo)
% This Animation code is for QuadCopter. Written by Jitendra Singh 

%% Define design parameters
D2R = pi/180;
R2D = 180/pi;
b   = 0.6;   % the length of total square cover by whole body of quadcopter in meter
a   = b/3;   % the legth of small square base of quadcopter(b/4)
H   = 0.06;  % hight of drone in Z direction (4cm)
H_m = H+H/2; % hight of motor in z direction (5 cm)
r_p = b/4;   % radius of propeller
%% Conversions
ro = 45*D2R;                   % angle by which rotate the base of quadcopter
Ri = [cos(ro) -sin(ro) 0;
      sin(ro) cos(ro)  0;
       0       0       1];     % rotation matrix to rotate the coordinates of base 
base_co = [-a/2  a/2 a/2 -a/2; % Coordinates of Base 
           -a/2 -a/2 a/2 a/2;
             0    0   0   0];
base = Ri*base_co;             % rotate base Coordinates by 45 degree 

to = linspace(0, 2*pi);
xp = r_p*cos(to);
yp = r_p*sin(to);
zp = zeros(1,length(to));
%% Define Figure plot
 fig1 = figure('pos', [0 50 800 600]);
 hg   = gca;
 view(68,53);
 grid on;
 axis equal;
 % xlim([-1.5 1.5]); ylim([-1.5 1.5]); zlim([0 3.5]);
 xlim([-2 8]); ylim([-5 5]); zlim([-2 5]);
 title('(JITENDRA) Drone Animation')
 xlabel('X[m]');
 ylabel('Y[m]');
 zlabel('Z[m]');
 hold(gca, 'on');
 
%% Design Different parts
% design the base square
 drone(1) = patch([base(1,:)],[base(2,:)],[base(3,:)],'r');
 drone(2) = patch([base(1,:)],[base(2,:)],[base(3,:)+H],'r');
 alpha(drone(1:2),0.7);
% design 2 parpendiculer legs of quadcopter 
 [xcylinder ycylinder zcylinder] = cylinder([H/2 H/2]);
 drone(3) =  surface(b*zcylinder-b/2,ycylinder,xcylinder+H/2,'facecolor','b');
 drone(4) =  surface(ycylinder,b*zcylinder-b/2,xcylinder+H/2,'facecolor','b') ; 
 alpha(drone(3:4),0.6);
% design 4 cylindrical motors 
 drone(5) = surface(xcylinder+b/2,ycylinder,H_m*zcylinder+H/2,'facecolor','r');
 drone(6) = surface(xcylinder-b/2,ycylinder,H_m*zcylinder+H/2,'facecolor','r');
 drone(7) = surface(xcylinder,ycylinder+b/2,H_m*zcylinder+H/2,'facecolor','r');
 drone(8) = surface(xcylinder,ycylinder-b/2,H_m*zcylinder+H/2,'facecolor','r');
 alpha(drone(5:8),0.7);
% design 4 propellers
 drone(9)  = patch(xp+b/2,yp,zp+(H_m+H/2),'c','LineWidth',0.5);
 drone(10) = patch(xp-b/2,yp,zp+(H_m+H/2),'c','LineWidth',0.5);
 drone(11) = patch(xp,yp+b/2,zp+(H_m+H/2),'p','LineWidth',0.5);
 drone(12) = patch(xp,yp-b/2,zp+(H_m+H/2),'p','LineWidth',0.5);
 alpha(drone(9:12),0.3);

%% create a group object and parent surface
  combinedobject = hgtransform('parent',hg );
  set(drone,'parent',combinedobject)
%  drawnow

% draw the gate
% gate_center = [3, 0, 2];
% gate_length = 0.8;
% gate_width  = 0.6; 
gate_center_flu = [gate.center(1), -gate.center(2), -gate.center(3)];
gate_vertice0= [ 0, -gate.width/2.0,  gate.length/2.0;
                 0,  gate.width/2.0,  gate.length/2.0;
                 0,  gate.width/2.0, -gate.length/2.0;
                 0, -gate.width/2.0, -gate.length/2.0]; % defined in foward-left-up frame

gate_vertice = rotx(-(90-gate.roll*180/pi)) * gate_vertice0' + gate_center_flu'; 
gate_vertice = gate_vertice'; 

line(gate_vertice([1:4,1],1), gate_vertice([1:4,1],2), gate_vertice([1:4,1],3));
 
count_frame = 1; 
 for i = 1:playspeed:length(x)
  
     ba = plot3(x(1:i),y(1:i),z(1:i), 'b:','LineWidth',1.5);
   
     translation = makehgtform('translate',...
                               [x(i) y(i) z(i)]);
     %set(combinedobject, 'matrix',translation);
     rotation1 = makehgtform('xrotate',(pi/180)*(roll(i)));
     rotation2 = makehgtform('yrotate',(pi/180)*(pitch(i)));
     rotation3 = makehgtform('zrotate',(pi/180)*yaw(i));
     %scaling = makehgtform('scale',1-i/20);
     set(combinedobject,'matrix',...
          translation*rotation3*rotation2*rotation1);
      
      view([-55 26])
      
     if (bRecordVideo==1) 
        movieVector(count_frame) =  getframe(fig1);
        count_frame = count_frame + 1;
     end 
        
       %delete(b);
     drawnow
   % pause(0.2);
   
   
 end

if bRecordVideo == 1
     v = VideoWriter('fly_gate.avi');
     open(v);
     writeVideo(v,movieVector);
     close(v);
end
 
