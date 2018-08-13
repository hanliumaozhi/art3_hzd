counter = 1;
control_point = zeros(1, 6);
for i = 1:4:24
    control_point(counter) = gait(1).params.aposition(i);
    counter = counter + 1;
end

time_arr = zeros(1, 101);
point_arr = zeros(1, 101);
nn_arr = zeros(1, 101);
for i = 0:100
    point_arr(1, i+1) = bezier(i*0.01, control_point);
    time_arr(1, i+1) = 0.01*i;
    nn_arr(1, i+1) = feval(sprintf('sagittal_library_q%i', 6), ([0.01*i; 0.8]));
end

plot(time_arr(1, :), point_arr(1, :), time_arr(1, :), nn_arr(1, :));