%%%  PosControl_Sim
% clear
% path('./icon/',path);
Init;

% constant value
RAD2DEG = 57.2957795;
DEG2RAD = 0.0174533;
CONSTANTS_ONE_G = 9.8; 
FLT_EPSILON = 1E-5; 

% throttle when UAV is hovering
% THR_HOVER = 0.609;
THR_HOVER = 0.331;

%% control parameter
% attitude PID parameters
% Kp_PITCH_ANGLE = 6.5;
% Kp_PITCH_AngleRate = 0.1;
% Ki_PITCH_AngleRate = 0.02;
% Kd_PITCH_AngleRate = 0.001;
% Kp_ROLL_ANGLE = 6.5;
% Kp_ROLL_AngleRate = 0.1*1.0;
% Ki_ROLL_AngleRate = 0.02;
% Kd_ROLL_AngleRate = 0.001;
% Kp_YAW_AngleRate = 0.5;
% Ki_YAW_AngleRate = 0.01;
% Kd_YAW_AngleRate = 0.00;
% Kp_YAW_ANGLE = 2.8;

% MC_PITCH_P = 6.5*2;
% MC_PITCHRATE_P = 0.15*10;
% MC_PITCHRATE_I = 0.2*0.1;
% MC_PITCHRATE_D = 0.003*2; 
% % MC_ROLL_P = 6.5;
% MC_ROLL_P = 6.5*1.5*2;
% % MC_ROLL_P = 11.5;
% MC_ROLLRATE_P = 0.15*1.5;
% MC_ROLLRATE_I = 0.2;
% MC_ROLLRATE_D = 0.003; 
% MC_YAW_P = 2.8;
% MC_YAWRATE_P = 0.2;
% MC_YAWRATE_I = 0.1;
% MC_YAWRATE_D = 0.00; 

% nuswarm param (min_snap_30_11_23)
MC_PITCH_P = 6.5;
MC_PITCHRATE_P = 0.06;
MC_PITCHRATE_I = 0.2;
MC_PITCHRATE_D = 0.0005; 

MC_ROLL_P = 11.5;
MC_ROLLRATE_P = 0.055;
MC_ROLLRATE_I = 0.2;
MC_ROLLRATE_D = 0.0005;

MC_YAW_P = 2.8;
MC_YAWRATE_P = 0.2;
MC_YAWRATE_I = 0.1;
MC_YAWRATE_D = 0.00; 


% position PID parameters
% Kpxp = 1.0;
% Kpyp = 1.0;
% Kpzp = 4.0*0.25;
% Kvxp = 2.5; Kvxi = 0.4; Kvxd = 0.01;
% Kvyp = 2.5; Kvyi = 0.4; Kvyd = 0.01;
% Kvzp = 0.45; Kvzi = 0.01; Kvzd = 0.005;


% MPC_XY_P = 0.95;
% MPC_XY_VEL_P_ACC = 1.8;
% MPC_XY_VEL_I_ACC = 0.4;
% MPC_XY_VEL_D_ACC = 0.2;
% 
% MPC_Z_P = 1; 
% MPC_Z_VEL_P_ACC = 4.0; 
% MPC_Z_VEL_I_ACC = 2.0; 
% MPC_Z_VEL_D_ACC = 0.0; 

% nuswarm param (min_snap_30_11_23)
MPC_XY_P = 2.2;
MPC_XY_VEL_P_ACC = 4.0;
MPC_XY_VEL_I_ACC = 0.4;
MPC_XY_VEL_D_ACC = 0.2;

MPC_Z_P = 0.8; 
MPC_Z_VEL_P_ACC = 4.0; 
MPC_Z_VEL_I_ACC = 2.0; 
MPC_Z_VEL_D_ACC = 0.0; 


% integral saturation
Saturation_I_RP_Max = 0.3;
Saturation_I_RP_Min = -0.3;
Saturation_I_Y_Max = 0.2;
Saturation_I_Y_Min = -0.2;
Saturation_I_ah = 3.43;
Saturation_I_az = 5;

% max control angle, default 35deg
% MAX_CONTROL_ANGLE_ROLL = 35;
% MAX_CONTROL_ANGLE_PITCH  = 35;
MAX_CONTROL_ANGLE_ROLL = 90;
MAX_CONTROL_ANGLE_PITCH  = 70;
% max control angle rate, rad/s 
MAX_CONTROL_ANGLE_RATE_PITCH = 220;
MAX_CONTROL_ANGLE_RATE_ROLL = 220;
MAX_CONTROL_ANGLE_RATE_YAW = 200;
MAX_CONTROL_ANGLE_RATE_Y = 200;
% max control speed, m/s
MAX_CONTROL_VELOCITY_XY = 5*2;
MAX_CONTROL_VELOCITY_Z = 3*2;
% throttle amplitude
MAX_MAN_THR = 0.9;
MIN_MAN_THR = 0.05;


FLT_EPSILON = 1e-5; 
MC_YAW_WEIGHT = 0.4; 
MPC_TILTMAX_AIR = 90; % default 45  in degree

MPC_THR_MAX = 0.9; 
MPC_THR_MIN = 0.06; 
MPC_THR_XY_MARG = 0.3;


%% xy outer-loop controller
wn    = 0.4;
sigma = 1.1*1.5;
ki    = 0.8*1.5;
eps   = 1*0.4;
F_xy  = [(wn^2+2*sigma*wn*ki)/eps^2, (2*sigma*wn+ki)/eps, ki*wn^2/eps^3, -(wn^2+2*sigma*wn*ki)/eps^2, -(2*sigma*wn+ki)/eps];

%% z outer-loop controller
wn    = 0.5;
% wn    = 0.7;
sigma = 1.1*1.5;
eps   = 1*0.5;
F_z   = [(wn^2+2*sigma*wn*ki)/eps^2, (2*sigma*wn+ki)/eps, ki*wn^2/eps^3, -(wn^2+2*sigma*wn*ki)/eps^2, -(2*sigma*wn+ki)/eps];



%% run simulink model
% PosControl_Sim_RPT