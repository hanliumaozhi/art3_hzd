% gaits postprocessing
% by jun 20180514
% modify the gaits, meet the demands : 4-bars & hip_roll == 0

%% 
% load optimal gait (and parameters)
clear;clc;
param = load('local/tmp_gait.mat');

% modify the gaits
for i = 1:1:4
    
    if mod(i,2) == 1
        ti = length(param.gait(i).tspan);
    else
        ti = 1;
    end

    % x
    param.gait(i).states.x(4,:) = zeros(1,ti);
    param.gait(i).states.x(9,:) = zeros(1,ti);
    param.gait(i).states.x(7,:) = param.gait(i).states.x(6,:);
    param.gait(i).states.x(8,:) = -param.gait(i).states.x(6,:);
    param.gait(i).states.x(12,:) = param.gait(i).states.x(11,:);
    param.gait(i).states.x(13,:) = -param.gait(i).states.x(11,:);
    % dx
    param.gait(i).states.dx(4,:) = zeros(1,ti);
    param.gait(i).states.dx(9,:) = zeros(1,ti);
    param.gait(i).states.dx(7,:) = param.gait(i).states.dx(6,:);
    param.gait(i).states.dx(8,:) = -param.gait(i).states.dx(6,:);
    param.gait(i).states.dx(12,:) = param.gait(i).states.dx(11,:);
    param.gait(i).states.dx(13,:) = -param.gait(i).states.dx(11,:);
    
    if mod(i,2) == 1
        
        % ddx
        param.gait(i).states.ddx(4,:) = zeros(1,ti);
        param.gait(i).states.ddx(9,:) = zeros(1,ti);
        param.gait(i).states.ddx(7,:) = param.gait(i).states.ddx(6,:);
        param.gait(i).states.ddx(8,:) = -param.gait(i).states.ddx(6,:);
        param.gait(i).states.ddx(12,:) = param.gait(i).states.ddx(11,:);
        param.gait(i).states.ddx(13,:) = -param.gait(i).states.ddx(11,:);
    else
        
        % xn
        param.gait(i).states.xn(4,:) = zeros(1,ti);
        param.gait(i).states.xn(9,:) = zeros(1,ti);
        param.gait(i).states.xn(7,:) = param.gait(i).states.xn(6,:);
        param.gait(i).states.xn(8,:) = -param.gait(i).states.xn(6,:);
        param.gait(i).states.xn(12,:) = param.gait(i).states.xn(11,:);
        param.gait(i).states.xn(13,:) = -param.gait(i).states.xn(11,:);
        
        % dxn
        param.gait(i).states.dxn(4,:) = zeros(1,ti);
        param.gait(i).states.dxn(9,:) = zeros(1,ti);
        param.gait(i).states.dxn(7,:) = param.gait(i).states.dxn(6,:);
        param.gait(i).states.dxn(8,:) = -param.gait(i).states.dxn(6,:);
        param.gait(i).states.dxn(12,:) = param.gait(i).states.dxn(11,:);
        param.gait(i).states.dxn(13,:) = -param.gait(i).states.dxn(11,:);
    end
    
end


%% save
save('local/sim_gait.mat','param');