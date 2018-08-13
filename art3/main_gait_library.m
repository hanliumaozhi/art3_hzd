% Main script for art3 gait library

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

delay_set = true;
  
load_path   = 'gen/sym'; % path to export binary Mathematica symbolic expression (MX) files
utils.init_path(load_path);

%% load robot model
% load the robot model
robot = sys.LoadModel(urdf, load_path, delay_set);  % call  loadDynamics()

% load hybrid system
system = sys.LoadSystem(robot, load_path);

%% Load optimization problem
% get the boundary values, needs to be manually set all boundaries.
bounds = opt.GetBounds(robot, [0.8, 0],0.4);

% load problem
nlp = opt.LoadProblem(system, bounds, load_path);

%% gen gait library 
target_vx = [0.8, 0.6, 0.4, 0.3, 0.2, 0.1, 0, -0.1, -0.2, -0.3, -0.4, -0.6, -0.8];

T = 0.4;
subfolder_name = 'library';
if ~exist(fullfile('local', subfolder_name), 'dir')
    mkdir(fullfile('local', subfolder_name));
end

counter = 1;

param = load('local/tmp_gait.mat');

for vx = target_vx
    
    speed = [vx, 0.0];
    bounds = opt.GetBounds(robot, speed, T);
    opt.updateVariableBounds(nlp, bounds);
    % update initial condition
    opt.updateInitCondition(nlp,param.gait);
        
    diary_name = fullfile('local', subfolder_name, sprintf('gait_X%0.1f.txt', vx));
    diary(diary_name);
    
    [gait, sol, info] = opt.solve(nlp, param.sol, param.info);
    
    pause(1);
        
    if info.status == 0 || info.status == 1
       data_name = fullfile('local', subfolder_name, sprintf('gait_X%0.1f.mat', vx));
       fprintf('Saving gait %s\n', data_name);
    else
       data_name = fullfile('local', subfolder_name, sprintf('gait_X%0.1f_FAILED.mat', vx));
       fprintf('Saving (failed) gait %s\n', data_name);
    end
    pause(0.2);
    diary off;
    pause(1);
    save(data_name, 'gait', 'sol', 'info', 'bounds', 'speed');

    counter = counter + 1;
end