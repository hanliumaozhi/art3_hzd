%% Generate best fit function
start_vx = [-0.8, -0.7, -0.6, -0.5, -0.4, -0.3, -0.2, -0.1, 0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8];
%target_vx = [0];
T = 0.4;
subfolder_name = 'library';
if ~exist(fullfile('local', subfolder_name, 'transition_fitting'), 'dir')
    mkdir(fullfile('local', subfolder_name, 'transition_fitting'));
end

N_vx = 62;
N_vy = 1;
N = 14;
j = 1;
t_all = cell(N_vx, N_vy);
q_all = cell(N_vx, N_vy);
dq_all = cell(N_vx, N_vy);
sx_all = cell(N_vx, N_vy);
tx_all = cell(N_vx, N_vy);
argument_x = cell(N_vx, N_vy);
argument_dx = cell(N_vx, N_vy);
index_map = [1, 3, 5, 7];
tmp_index = 1;
counter = 1;
for i=1:size(start_vx, 2)
    for tmp_i = -2:2
        if tmp_i ~= 0 && (i+tmp_i <= size(start_vx, 2)) && (i+tmp_i >= 1)
            target_vx = start_vx(tmp_i+i);
            data_name = fullfile('local', subfolder_name, 'transition', ...
                sprintf('gait_X%0.1f_Y%.1f_TO_X%0.1f_Y%.1f.mat', start_vx(i), 0.0, target_vx, 0.0));
            param = load(data_name);
            
            data_name = fullfile('local', subfolder_name, sprintf('gait_X%0.1f.mat', start_vx(i)));
            s_gait = load(data_name);
        
            tmp_list = param.gait(1).tspan-param.gait(1).tspan(11);
            t_all{i,j} = tmp_list(11:21);%, param.gait(3).tspan, param.gait(5).tspan, param.gait(7).tspan]./(4*T);%, param.gait(3).tspan, param.gait(5).tspan, param.gait(7).tspan]./(4*T);%, param.gait(3).tspan];
            q_all{i,j} = param.gait(1).states.x(:, 11:21);%, param.gait(3).states.x, param.gait(5).states.x, param.gait(7).states.x];
            dq_all{i,j} = param.gait(1).states.dx(:, 11:21);%, param.gait(3).states.dx, param.gait(5).states.dx, param.gait(7).states.dx];
            %sx_all{i,j} = ones(size(t_all{i,j}))*start_vx(i);%ones(size(t_all{i,j}))*vx;
            tx_all{i,j} = ones(size(t_all{i,j}))*target_vx;%ones(size(t_all{i,j}))*vy
            argument_x{i, j} = param.gait(1).states.x(1, 11:21);
            argument_dx{i, j} = param.gait(1).states.dx(1, 11:21);
        end
    end
end

inpt = [horzcat(t_all{:})
    horzcat(tx_all{:})
    %horzcat(argument_x{:})
    horzcat(argument_dx{:})];
    %horzcat(phase_all{:})];
q_outpt = horzcat(q_all{:});
dq_outpt = horzcat(dq_all{:});
%%
disp(size(q_outpt));
disp(size(dq_outpt));
act_list = [6:7, 11:12];
for i = act_list(1, :)
    q_outpt_specific = q_outpt(i, :);
    dq_outpt_specific = dq_outpt(i, :);
    opt.neuralFitting(q_outpt_specific, inpt, 5, fullfile('local', subfolder_name, 'transition_fitting', sprintf('sagittal_library_q%i', i)));
    opt.neuralFitting(dq_outpt_specific, inpt, 5, fullfile('local', subfolder_name, 'transition_fitting', sprintf('sagittal_library_dq%i', i)));
end
addpath(fullfile('local', subfolder_name, 'transition_fitting'));
%% Plot features
% joint angle
joint_names = { 'px'
    'pz'
    'r'
    'pelvis_fixed'
    'l_hip_roll'
    'l_hip_pitch'
    'l_knee_pitch'
    'l_FB_1'
    'l_FB_2'
    'r_hip_roll'
    'r_hip_pitch'
    'r_knee_pitch'
    'r_FB_1'
    'r_FB_2'};

for n = 1:N
    f = figure(1000+n);
    f.Name = joint_names{n};
    set(f, 'WindowStyle', 'docked');
    
    ax = axes(f);
    hold(ax);
    
    scatter3(ax, inpt(1,:),inpt(2,:),q_outpt(n,:), 'r');
    
    [S, V] = meshgrid(0:0.01:1, -2:0.04:2);
    L = zeros(size(S));
    for i = 1:size(S, 1)
        for j = 1:size(S, 2)
            L(i, j) = feval(sprintf('sagittal_library_q%i', n), ([S(i, j); V(i, j)]));
            %L(i, j) = l(n);
        end
    end
    
    surface(ax, S, V, L);
end
%% joint velocity
for n = 1:N
    f = figure(2000+n);
    f.Name = joint_names{n};
    set(f, 'WindowStyle', 'docked');
    
    ax = axes(f);
    hold(ax);
    
    scatter3(ax, inpt(1,:),inpt(2,:),dq_outpt(n,:), 'r');
    
    [S, V] = meshgrid(0:0.01:1, -2:0.01:2);
    L = zeros(size(S));
    for i = 1:size(S, 1)
        for j = 1:size(S, 2)
            L(i, j) = feval(sprintf('sagittal_library_dq%i', n), ([S(i, j); V(i, j)]));
        end
    end
    
    surface(ax, S, V, L);
end