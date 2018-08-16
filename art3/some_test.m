subfolder_name = 'library';

vec_index = 0.4;
data_name = fullfile('local', subfolder_name, ...
                sprintf('gait_X%0.1f.mat', vec_index));
param = load(data_name);

counter = 1;
control_point = zeros(1, 6);
for i = 1:4:24
    control_point(counter) = param.gait(1).params.aposition(i);
    counter = counter + 1;
end

addpath(fullfile('local', subfolder_name, 'periodic_fitting'));

time_arr = zeros(1, 101);
point_arr = zeros(1, 101);
nn_arr = zeros(1, 101);
for i = 0:100
    point_arr(1, i+1) = bezier_vec(i*0.01, control_point, 0.4);
    time_arr(1, i+1) = 0.01*i;
    nn_arr(1, i+1) = feval(sprintf('sagittal_library_dq%i', 6), ([0.01*i; vec_index]));
end

plot(time_arr(1, :), point_arr(1, :), 'b', time_arr(1, :), nn_arr(1, :), 'r');