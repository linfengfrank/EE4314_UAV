%% Minimum-Snap Trajectory Throght a Gate 
% Author: Dr Feng LIN
% Description: Geneare a 3D minimum-snap trajectry that passes a gate

%% Clear the workspace
clc; clear; close all;
addpath funct

% Configuration
% Using NED coordinate system to define the constraints and trajectory
% the ground height is 0
g = 9.781;

% Gate paramters (NED)
gate.center = [3, -2, -1]; % in NED frame in meters
gate.roll   = deg2rad(60);  % roll angle of the gate is 60 degree !!!
gate.length = 0.8;         % gate x-size
gate.width  = 0.6;         % gate y-size

% start and end points
start_point = [0,  0,  0];
end_point   = [5, -2, -1]; 


% [To be completed] Define acceleration constraints based on the roll angle requirement
acx_constrain = ;
acy_constrain = ;
acz_constrain = ; 


%% Waypoints
% Start -> Gate center -> Hovering at end point
path =[ start_point;
        gate.center;
        end_point]; % in NED frame

    
% Polynamial order and segments
n_order = 7;                    % per-segment polynomial order (≥7 recommended)
n_seg   = size(path, 1) - 1;
n_poly_perseg = n_order + 1;


%% Time allocation (per segment) using trapezoidal speed profile
vel_max = 3.0;
acc_max = 5.0;

t_slope = vel_max/ acc_max;
max_s_triangle = t_slope * vel_max;

ts = zeros(n_seg, 1);
for i=1:n_seg
    s = sqrt((path(i+1, 1) - path(i, 1))^2 + (path(i+1, 2) - path(i, 2))^2 + (path(i+1, 3) - path(i, 3))^2 );
    
    if( s<= max_s_triangle)
       ts(i) = 2.0 * s / acc_max; 
    else
       ts(i) = (s-max_s_triangle) / vel_max + t_slope; 
    end
end
     
ts = ts * 1.5; 

%% Minimum-snap solve per axis
poly_coef_x = solveMinimumSnapQP(path(:, 1), ts, n_seg, n_order, acx_constrain);
poly_coef_y = solveMinimumSnapQP(path(:, 2), ts, n_seg, n_order, acy_constrain);
poly_coef_z = solveMinimumSnapQP(path(:, 3), ts, n_seg, n_order, acz_constrain);


%% Sample trajectory + attitude
k = 1;
tstep = 0.01;
zero_vec = zeros(floor(sum(ts/tstep)),1); 

X_n     = zero_vec;  
Y_n     = zero_vec; 
Z_n     = zero_vec;
Vx_n    = zero_vec;  
Vy_n    = zero_vec; 
Vz_n    = zero_vec;
Ax_n    = zero_vec;  
Ay_n    = zero_vec; 
Az_n    = zero_vec; 
Yaw     = zero_vec; 
Yawrate = zero_vec;

Roll    = zero_vec; 
Pitch   = zero_vec; 

for i=0:n_seg-1
    %#####################################################
    % get the coefficients of i-th segment of both x-axis
    % and y-axis and z-axis
    
    % position
    Pxi = flip( poly_coef_x( i*(n_order+1) + 1 : (i+1)*(n_order+1) ) );
    Pyi = flip( poly_coef_y( i*(n_order+1) + 1 : (i+1)*(n_order+1) ) );
    Pzi = flip( poly_coef_z( i*(n_order+1) + 1 : (i+1)*(n_order+1) ) );
    
    % velocity
    Pvxi = polyder(Pxi);
    Pvyi = polyder(Pyi);
    Pvzi = polyder(Pzi);
    
    % acceleration
	Paxi = polyder(Pvxi);
	Payi = polyder(Pvyi);
    Pazi = polyder(Pvzi);
    
    for t=0:tstep:ts(i+1)
        X_n(k)  = polyval(Pxi,t);
        Y_n(k)  = polyval(Pyi,t);
        Z_n(k)  = polyval(Pzi,t);
        
        Vx_n(k)  = polyval(Pvxi,t);
        Vy_n(k)  = polyval(Pvyi,t);
        Vz_n(k)  = polyval(Pvzi,t);
        
        Ax_n(k)  = polyval(Paxi,t);
        Ay_n(k)  = polyval(Payi,t);
        Az_n(k)  = polyval(Pazi,t);
        
        % simplified method to compute Roll and Pitch in small angle
        % Roll(k)  = atan2( -(- Ay_n(k)), -( Az_n(k) - g ) );
        % Pitch(k) = atan2( - Ax_n(k), -( Az_n(k) - g ) );
        
        % compute roll and pitch using rotation matrix
        % Z_B = [Ax_n(k), Ay_n(k), -Az_n(k)-g]';
        
        Z_B = -[Ax_n(k), Ay_n(k), Az_n(k)-g]';
        Z_B = Z_B / norm(Z_B);
        
        Yaw(k) = 0; 
        Yawrate(k) = 0; 
        X_C = [cos(0), sin(0), 0]';
        
        Y_B = cross(Z_B, X_C);
        Y_B = Y_B / norm(Y_B);
        
        X_B = cross(Y_B, Z_B);
        
        R_B = [X_B, Y_B, Z_B];
        
        eul = rotm2eul(R_B, 'ZYX'); 
        
        Roll(k) = eul(3);
        Pitch(k) = eul(2);
        
        k = k+1;
    end
end


%% 3D Path (NED)
figure(); 
plot3(X_n, Y_n, Z_n, 'Color',[0 1.0 0],'LineWidth',2);
hold on

scatter3(path(1:size(path,1),1),path(1:size(path,1),2),path(1:size(path,1),3)); grid on;
xlabel('North'); ylabel('East'); zlabel('Down'); axis equal; 

% animate by using the function in FLU
playspeed = 5;
bRecordVideo = 0; 
drone_gate_Animation(X_n, -Y_n, -Z_n, Roll*180/pi, -Pitch*180/pi, -Yaw*180/pi, gate, playspeed, bRecordVideo); % need to convert NED to FLU frame



%% plot the states of the generated trajectory 
t_total = ( 0: tstep : ( size(Vx_n,1)-1 )*tstep )'; 
traj_state.time = t_total; 
traj_state.X_n = X_n;
traj_state.Y_n = Y_n;
traj_state.Z_n = Z_n;

traj_state.Vx_n = Vx_n;
traj_state.Vy_n = Vy_n;
traj_state.Vz_n = Vz_n;

traj_state.Ax_n = Ax_n;
traj_state.Ay_n = Ay_n;
traj_state.Az_n = Az_n;

traj_state.Roll  = Roll;
traj_state.Pitch = Pitch;
traj_state.Yaw = Yaw;

plotStates(traj_state, ts, 'States of generated trajectory'); 


%% Integration with the Simulink Model with flight control
addpath position_control;
addpath position_control/icon;

ref.time = t_total;
% [To be completed] Input the trajectory reference into the Simulink model.
ref.signals.values = [ ];
ref.signals.dimensions = 11; 

% Init_control; % It will be run at InitFcn* at Model Explorer

% Load model
model = 'PosControl_Sim_RPT';
% load_system(model);  % Model is in memory but not visible (Faster)
open_system(model);    % Opens model visually in Simulink


% Use the last time from your trajectory as the stop time
set_param(model, 'StopTime', num2str(t_total(end)));

% conduct the simulation in Simulink
sim(model); 

% read the output of the simulaiton
sim_pose.t = PosE.time; % time
sim_pose.x = PosE.signals.values(:,1); % position
sim_pose.y = PosE.signals.values(:,2);
sim_pose.z = PosE.signals.values(:,3);

sim_ang.roll  = AngEuler.signals.values(1,1,:); % Euler angle
sim_ang.pitch = AngEuler.signals.values(1,2,:); 
sim_ang.yaw   = AngEuler.signals.values(1,3,:);


%% Play animation
playspeed    = 40;
bRecordVideo = 0; 
drone_gate_Animation(sim_pose.x, -sim_pose.y, -sim_pose.z, sim_ang.roll*180/pi, -sim_ang.pitch*180/pi, -sim_ang.yaw*180/pi, gate, playspeed, bRecordVideo); % need to convert NED to FLU frame





%% ========================== Functions ===============================
function poly_coef = solveMinimumSnapQP(waypoints, ts, n_seg, n_order, acc_constraint)
    start_cond = [waypoints(1), 0, 0, 0];
    end_cond   = [waypoints(end), 0, 0, 0];

    Q = getQ(n_seg, n_order, ts);                          % Cost matrix
    [Aeq, beq] = getAbeq(n_seg, n_order, waypoints, ts, ...
        start_cond, end_cond, acc_constraint);            % Equality constraints
    f = zeros(size(Q,1), 1);                               % Linear term
    poly_coef = quadprog(Q, f, [], [], Aeq, beq);          % Solve QP
end


function plotStates(state, ts, title)
% plot the states
% state: the states of the uav
% ts: time of each segment
% title: the title of the plot

t_total = state.time;
X_n = state.X_n;
Y_n = state.Y_n;
Z_n = state.Z_n;

Vx_n = state.Vx_n;
Vy_n = state.Vy_n;
Vz_n = state.Vz_n;

Ax_n = state.Ax_n;
Ay_n = state.Ay_n;
Az_n = state.Az_n;

Roll  = state.Roll;
Pitch = state.Pitch;
Yaw   = state.Yaw;

figure;
subplot(3,4,1);
plot(t_total, X_n); grid on; ylabel('x'); line([ts(1), ts(1)], [min(X_n), max(X_n)], 'Color', [1 0 0]);
subplot(3,4,5);
plot(t_total, Y_n); grid on; ylabel('y'); line([ts(1), ts(1)], [min(Y_n), max(Y_n)], 'Color', [1 0 0]);
subplot(3,4,9);
plot(t_total, Z_n); grid on; ylabel('z'); line([ts(1), ts(1)], [min(Z_n), max(Z_n)], 'Color', [1 0 0]);

subplot(3,4,2);
plot(t_total, Vx_n); grid on; ylabel('vx'); line([ts(1), ts(1)], [min(Vx_n), max(Vx_n)], 'Color', [1 0 0]);
subplot(3,4,6);
plot(t_total, Vy_n); grid on; ylabel('vy'); line([ts(1), ts(1)], [min(Vy_n), max(Vy_n)], 'Color', [1 0 0]);
subplot(3,4,10);
plot(t_total, Vz_n); grid on; ylabel('vz'); line([ts(1), ts(1)], [min(Vz_n), max(Vz_n)], 'Color', [1 0 0]);

subplot(3,4,3);
plot(t_total, Ax_n); grid on; ylabel('ax'); line([ts(1), ts(1)], [min(Ax_n), max(Ax_n)], 'Color', [1 0 0]);
subplot(3,4,7);
plot(t_total, Ay_n); grid on; ylabel('ay'); line([ts(1), ts(1)], [min(Ay_n), max(Ay_n)], 'Color', [1 0 0]);
subplot(3,4,11);
plot(t_total, Az_n); grid on; ylabel('az'); line([ts(1), ts(1)], [min(Az_n), max(Az_n)], 'Color', [1 0 0]);

subplot(3,4,4);
plot(t_total, Roll); grid on; ylabel('roll'); line([ts(1), ts(1)], [min(Roll), max(Roll)], 'Color', [1 0 0]);
subplot(3,4,8);
plot(t_total, Pitch); grid on; ylabel('picth'); line([ts(1), ts(1)], [min(Pitch), max(Pitch)], 'Color', [1 0 0]);
subplot(3,4,12);
plot(t_total, Yaw); grid on; ylabel('yaw'); line([ts(1), ts(1)], [min(Yaw), max(Yaw)], 'Color', [1 0 0]);

% Add overall title
sgtitle(title);

end