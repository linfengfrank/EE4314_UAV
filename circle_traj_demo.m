%% circle_traj_demo
% Constant-speed circular trajectory for a multirotor in NED.
% Theta(t) = theta0 + omega * t.
% Exports 'ref' (11 signals) and timeseries for Simulink.

clc; close all;
addpath funct

%% ===== User params =====
dt        = 0.01;           % sample time (s)
T_end     = 15.0;           % end time (s)
dir_sign  = +1;             % +1 CCW, -1 CW (top-down view)
R         = 2.0;            % radius (m)

center_N  =-2.0;            % circle center North (m)
center_E  = 2.0;            % circle center East  (m)
center_D  = 0.0;            % constant Down (m)  

yaw_mode  = "tangent";      % "tangent" or "fixed"
yaw_fixed = deg2rad(0);     % used when yaw_mode="fixed"

%% ===== Time & angle =====
t       = (0:dt:T_end).';
theta0  = 0;                                        % start angle
omega   = 0.8;                                      % constant angular rate (rad/s)
theta   = theta0 + omega .* t;                      % angle vs time


%% ===== Position, velocity, acceleration (NED) =====
cN = center_N; cE = center_E; cD = center_D;

cosT = cos(theta);  sinT = sin(theta);

% Position
X_n = cN + R * cosT;
Y_n = cE + R * sinT;
Z_n = cD + zeros(size(t));

% Velocity (dp/dt) = (dp/dθ) * ω
Vx_n = -R * sinT * omega;
Vy_n =  R * cosT * omega;
Vz_n =  zeros(size(t));

% Acceleration (d2p/dt2) = (d2p/dθ2) * ω^2
Ax_n = -R * cosT * (omega^2);
Ay_n = -R * sinT * (omega^2);
Az_n =  zeros(size(t));

%% ===== Yaw & yaw rate =====
switch yaw_mode
    case "tangent"
        Yaw = atan2(Vy_n, Vx_n);            % face along velocity
    case "fixed"
        Yaw = yaw_fixed * ones(size(t));
    otherwise
        error('Unknown yaw_mode.');
end
Yawrate = [0; diff(Yaw)] ./ [1; diff(t)];   % simple finite diff


%% ===== Quick plots =====
figure('Name','Circle (NED)','Color','w');
plot3(X_n, Y_n, Z_n, 'LineWidth',2); grid on; axis equal;
hold on; plot3(cN,cE,cD,'ko','MarkerFaceColor','k');
xlabel('North (m)'); ylabel('East (m)'); zlabel('Down (m)');

title(sprintf('R=%.1f m, T=%.1f s, |v|=R|ω|=%.2f m/s', ...
      R, T_end, abs(R*omega)));



%% ========  Simulink integration  ==============
% conduct simulink simulation with flight control
addpath position_control;
addpath position_control/icon;

% Order: [x y z vx vy vz ax ay az yaw yawrate]
ref.time = t;
ref.signals.values     = [X_n Y_n Z_n Vx_n Vy_n Vz_n Ax_n Ay_n Az_n Yaw Yawrate];
ref.signals.dimensions = 11;

model = 'PosControl_Sim_RPT';
% load_system(model);
open_system(model);

% Use the last time from your trajectory as the stop time
set_param(model, 'StopTime', num2str(t(end)));

% condut the simulation 
sim(model); 

% read the output of the simulaiton
sim_pose.t = PosE.time; 
sim_pose.x = PosE.signals.values(:,1);
sim_pose.y = PosE.signals.values(:,2);
sim_pose.z = PosE.signals.values(:,3);


%% Plot the simulation results
figure;
subplot(2,2,1);
plot(sim_pose.t, sim_pose.x); grid on; 
xlabel('time (s)'); ylabel('x (m)');

subplot(2,2,2);
plot(sim_pose.t, sim_pose.y); grid on; 
xlabel('time (s)'); ylabel('y (m)');

subplot(2,2,3);
plot(sim_pose.t, sim_pose.y); grid on; 
xlabel('time (s)'); ylabel('z (m)');

subplot(2,2,4);
plot3(sim_pose.x, sim_pose.y, sim_pose.z, 'LineWidth',2); grid on; axis equal;
xlabel('North (m)'); ylabel('East (m)'); zlabel('Down (m)');