control_point = zeros(1, 6);
control_point(1) = 1.1755;
control_point(2) = 1.1729;
control_point(3) = 1.7404;
control_point(4) = 1.6319;
control_point(5) = 1.2217;
control_point(6) = 1.1739;

time_list = 0:0.05:1;
data_list = ones(1, 21);

counter = 1;
for i = 0:0.05:1
    data_list(counter) = bezier_vec(i, control_point, 0.4);
    counter = counter + 1;
end
data_list