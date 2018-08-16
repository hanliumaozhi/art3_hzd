%% Setting up path
clear; close all; clc;
%restoredefaultpath; matlabrc;   % Restore search path to factory-installed state, Start up file for MATLAB program.

% specify the path to the FROST
% only for my computer you need change it
frost_path  = '../../frost-dev';
addpath(frost_path);
frost_addpath; % initialize FROST
export_path = 'gen/trans/opt';
utils.init_path(export_path);

load_sym  = true; % if true, it will load symbolic expression from 
if load_sym    
    load_path = 'gen/trans/sym'; % path to export binary Mathematica symbolic expression (MX) files
    utils.init_path(load_path);
else
    load_path   = []; 
end

%% robot model settings
cur = utils.get_root_path();
urdf = fullfile(cur,'urdf','art3_description_new.urdf'); % Build full file name from parts 

delay_set = false;

%% load robot model
% load the robot model
robot = sys.LoadModel(urdf, load_path, delay_set);  % call  loadDynamics()

% load hybrid system
system = sys.LoadTransSystem(robot, load_path);

%% Load optimization problem
T = 0.4;
subfolder_name = 'library';
start_gait_file = fullfile('local', subfolder_name, 'gait_X0.0.mat');
start_gait = load(start_gait_file);
x0 = [start_gait.gait(1).states.x(:,11);start_gait.gait(1).states.dx(:,11)];

target_gait_file = fullfile('local', subfolder_name, 'gait_X0.0.mat');
target_gait = load(target_gait_file);
xf = [target_gait.gait(3).states.x(:,11);target_gait.gait(3).states.dx(:,11)];
vel_lb = [0, 0];
vel_ub = [0, 0];

bounds = trans_opt.GetBounds(robot, vel_lb, vel_ub, T, x0, xf);

%%
% load problem
param = load('local/tmp_gait.mat');
nlp = trans_opt.LoadProblem(system, bounds, param.gait, load_path);

%% Compile stuff if needed
compileObjective(nlp,[],[],export_path);
compileConstraint(nlp,[],[],export_path);
% compileConstraint(nlp,[],[],export_path);
% % Save expression 

load_path   = 'gen/trans/sym';
utils.init_path(load_path);
system.saveExpression(load_path); % run this after loaded the optimization problem

%% work
start_vx_series = {[0, 0.2, 0.4, 0.6, 0.8]
    [0, -0.2, -0.4, -0.6, -0.8]
    0
    0};
% start_vy = [0, -0.2, -0.4, -0.6, -0.8];
target_vx = [0.0];
start_vy_series = {0
    0
    [0,0.1,0.2,0.3,0.4]
    [0,-0.1,-0.2,-0.3,-0.4]};
target_vy = [0.0];

if ~exist(fullfile('local', subfolder_name, 'transition'), 'dir')
    mkdir(fullfile('local', subfolder_name, 'transition'));
end

fit_data = load(fullfile('local', subfolder_name, 'midstep_fit.mat'));
P = fit_data.P;
dP = fit_data.dP;

fit_data_right = load(fullfile('local', subfolder_name, 'midstep_fit_right.mat'));
P_right = fit_data_right.P;
dP_right = fit_data_right.dP;
% nlp.Phase(1).removeCost('stateDeviation_RightStance');
% nlp.Phase(3).removeConstraint('periodicState_LeftStance');
start_vx = [-0.8, -0.7, -0.6, -0.5, -0.4, -0.3, -0.2, -0.1, 0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8];
counter = 1;
for i=1:size(start_vx, 2)
    start_vy = 0.0;
    vy = 0.0;
    %start_vx = start_vx(1, i);
    
    for vx = -2:2
        if vx ~= 0 && (i+vx <= size(start_vx, 2)) && (i+vx >= 1)
            target_vx = start_vx(vx+i);
            target_gait_file = fullfile('local', subfolder_name, sprintf('gait_X%0.1f.mat', target_vx));
            target_gait = load(target_gait_file);
            guess = [target_gait.gait; target_gait.gait(1:3)];
            guess(3).tspan = guess(3).tspan - guess(3).tspan(1);
            guess(7).tspan = guess(7).tspan - guess(7).tspan(1);
            
            data_name = fullfile('local', subfolder_name, 'transition', ...
                sprintf('gait_X%0.1f_Y%.1f_TO_X%0.1f_Y%.1f.mat', start_vx(i), 0.0, target_vx, 0.0));
            if exist(data_name, 'file')
                continue;
            end
            
            vel_lb = [min(start_vx(i), target_vx), 0.0];
            vel_ub = [max(start_vx(i), target_vx), 0.0];
            
            
            
            start_gait_file = fullfile('local', subfolder_name, sprintf('gait_X%0.1f.mat', start_vx(i)));
            start_gait = load(start_gait_file);
            x0 = [start_gait.gait(1).states.x(:,11);start_gait.gait(1).states.dx(:,11)];
            
            target_gait_file = fullfile('local', subfolder_name, sprintf('gait_X%0.1f.mat', target_vx));
            target_gait = load(target_gait_file);
            xf = [target_gait.gait(3).states.x(:,11);target_gait.gait(3).states.dx(:,11)];
            
            bounds = trans_opt.GetBounds(robot, vel_lb, vel_ub, T, x0, xf);
            bounds.LeftStance1.midState_QFit = P;
            bounds.LeftStance1.midState_dQFit = dP;
            
            
            bounds.RightStance2.midState_QFit = P_right;
            bounds.RightStance2.midState_dQFit = dP_right;
            
    
            trans_opt.updateVariableBounds(nlp, bounds);
            %             nlp.Phase(3).removeConstraint('periodicState_LeftStance');
            % update desired gaits
            gait_cost = [target_gait.gait(1), target_gait.gait(3), target_gait.gait(1), target_gait.gait(3)];
            trans_opt.updateDesiredGait(nlp, system, gait_cost);
            
            % update initial condition
            trans_opt.updateInitCondition(nlp,guess);
                        
            diary_name = fullfile('local', subfolder_name, 'transition', ...
                sprintf('gait_X%0.1f_Y%.1f_TO_X%0.1f_Y%.1f.txt',  start_vx(i), 0.0, target_vx, 0.0));
            diary(diary_name);
            
            [gait, sol, info, total_time] = trans_opt.solve(nlp);
            pause(1);
            if info.status == 0 || info.status == 1
                data_name = fullfile('local', subfolder_name, 'transition', ...
                    sprintf('gait_X%0.1f_Y%.1f_TO_X%0.1f_Y%.1f.mat',  start_vx(i), 0.0, target_vx, 0.0));
                fprintf('Saving gait %s\n', data_name);
            else
                data_name = fullfile('local', subfolder_name, 'transition', ...
                    sprintf('gait_X%0.1f_Y%.1f_TO_X%0.1f_Y%.1f_Failed.mat',  start_vx(i), 0.0, target_vx, 0.0));
                fprintf('Saving (failed) gait %s\n', data_name);
            end
            pause(0.2);
            diary off;
            pause(1);
            save(data_name, 'gait', 'sol', 'info', 'bounds', 'total_time');
            
            
            counter = counter + 1;
            guess = gait;
        end
        
    end
end