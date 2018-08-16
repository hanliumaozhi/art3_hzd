subfolder_name = 'library';

% time
time_list = 0:0.02:0.2;


data_name = fullfile('local', subfolder_name, 'transition', ...
                sprintf('gait_X%0.1f_Y%.1f_TO_X%0.1f_Y%.1f.mat', 0.0, 0.0, 0.2, 0.0));
param = load(data_name);

data_list = ones(11, 1);
nn_data_list = ones(11, 1);

counter = 1;

joint_index = 6;

for i = 1:11
    data_list(counter, 1) = param.gait(1).states.x(joint_index, (i+10));
    counter = counter + 1;
end

counter = 1;
addpath(fullfile('local', subfolder_name, 'transition_fitting'));
for j = 0:0.02:0.2
    nn_data_list(counter, 1) = feval(sprintf('sagittal_library_q%i', joint_index), ([j; 0.2; param.gait(1).states.dx(1, (counter+10))]));%;param.gait(1).states.dx(1, (counter+10))]));
    counter = counter + 1;
end

%nn_data_list(counter) = feval(sprintf('sagittal_library_dq%i', joint_index), ([1.0; 0.2; 0.0; 4]));

plot(time_list', data_list, 'b', time_list', nn_data_list, 'r');