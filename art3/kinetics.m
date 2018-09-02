%% Setting up path
clear; close all; clc;
%restoredefaultpath; matlabrc;   % Restore search path to factory-installed state, Start up file for MATLAB program.

% specify the path to the FROST
% only for my computer you need change it
frost_path  = '../../frost-dev';
addpath(frost_path);
frost_addpath; % initialize FROST
export_path = 'gen/opt'; % path to export compiled C++ and MEX files
utils.init_path(export_path);
%% robot model settings
cur = utils.get_root_path();
urdf = fullfile(cur,'urdf','art3_description_new.urdf'); % Build full file name from parts 

% some options

% if 'delay_set' is true, the computation of system dynamics (Coriolis
% vector) will be delayed. Delaying this operation will save significant
% loading time.
delay_set = true;

% if 'load_sym' is true, it will load symbolic expressions from previously
% save external files instead of re-compute them. It reduce the loading
% time by 7-10 faster. 
% Set it to false for the first time, and save expressions after loaded the
% model. 
load_sym  = true; % if true, it will load symbolic expression from 
if load_sym    
    load_path = 'gen/sym'; % path to export binary Mathematica symbolic expression (MX) files
    utils.init_path(load_path);
else
    load_path = []; 
end

%% load robot model
% load the robot model
robot = sys.LoadModel(urdf, load_path, delay_set);  % call  loadDynamics()

% load hybrid system
system = sys.LoadSystem(robot, load_path);

%% kinetics
r_hip_p_frame = robot.Joints(getJointIndices(robot, 'r_hip_pitch'));
r_hip_p_p = getCartesianPosition(robot, r_hip_p_frame);

r_ankle_frame = robot.FB3_Points.r_FB3;
r_ankle = getCartesianPosition(robot, r_ankle_frame);

robot_state_var = ones(21, 1);

for i = 1:21
    xx = gait(1).states.x(:, i);
    r_hip_p_x = double(subs(r_hip_p_p, robot.States.x, xx));
    r_hip_p_x = r_hip_p_x(1);
    
    r_ankle_x = double(subs(r_ankle, robot.States.x, xx));
    r_ankle_x = r_ankle_x(1);
    
    robot_state_var(i, 1) = (r_hip_p_x - r_ankle_x);
end

%% save expression
r_hip_p_frame = robot.Joints(getJointIndices(robot, 'r_hip_pitch'));
r_hip_p_p = getCartesianPosition(robot, r_hip_p_frame);

r_ankle_frame = robot.FB3_Points.r_FB3;
r_ankle = getCartesianPosition(robot, r_ankle_frame);

p_to_right_bottom = SymFunction('p_to_right_bottom', r_ankle, robot.States.x);

my_export_path = 'gen/test_export';
utils.init_path(my_export_path);
export(p_to_right_bottom, my_export_path)