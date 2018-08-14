% Main script


%% Setting up path
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

%% initialize model settings
cur = utils.get_root_path();
urdf = fullfile(cur,'urdf','art3_description_new.urdf');
delay_set = false;
%% load robot model
tic
robot = sys.LoadModel(urdf, load_path, delay_set);

% load hybrid system
system = sys.LoadTransSystem(robot, load_path);


bounds = trans_opt.GetBounds(robot);

% load problem
param = load('local/good_gait.mat');
nlp = trans_opt.LoadProblem(system, bounds, param.gait, load_path);
toc
%% Compile stuff if needed
compileConstraint(nlp,[],[],export_path,{'dynamics_equation'});
compileObjective(nlp,[],[],export_path);
% compileConstraint(nlp,[],[],export_path);
% % Save expression 
load_path   = 'gen/sym';
system.saveExpression(load_path); % run this after loaded the optimization problem

%% gait library
param = load(fullfile('local','library','transition','gait_X0.2_Y0.0_TO_X0.0_Y0.0.mat'));
% trans_opt.updateVariableBounds(nlp, param.bounds);

% checkConstraints(nlp, param.sol, 1e-3, 'local/constr.txt')
% checkVariables(nlp, param.sol, 1e-3,'local/var.txt')
% 
% plot.plotOptTorques(robot,nlp,param.gait)
% plot.plotOptStates(robot,nlp,param.gait)

anim = plot.LoadAnimator(robot, param.gait,'SkipExporting',true);
%% update bounds
% bounds = opt.GetBounds(robot);
% opt.updateVariableBounds(nlp, bounds);
% % update initial condition
% param = load('local/good_gait.mat');
% 
% opt.updateInitCondition(nlp,param.gait);
% %% solve
% [gait, sol, info] = opt.solve(nlp, sol);
% 
% %% save
% save('local/good_gait.mat','gait','sol','info','bounds');

%% animation
anim = plot.LoadAnimator(robot, gait,'SkipExporting',true);