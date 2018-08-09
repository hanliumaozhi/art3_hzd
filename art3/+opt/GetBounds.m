function bounds = GetBounds(model, speed, T_)
    
    if nargin < 2
        speed = [0, 0]; % walk in place gait
    end
    
    if nargin < 3
        T_ = 0.5;    % time-consuming of single step (unit: s)
    end
    
    % Width of hip_rolls
    wt = 0.23;
    vx = speed(1);
    vy = speed(2);
    
    %% some common boundary values are defined here
    
    % first get the model specific boundary values
    model_bounds = model.getLimits(); % x, dx, ddx, u
    
    model_bounds.inputs.Control.u.lb(3) = -130;
    model_bounds.inputs.Control.u.lb(8) = -130;
    model_bounds.inputs.Control.u.ub(3) = 130;
    model_bounds.inputs.Control.u.ub(8) = 130;
    
    model_bounds.states.x.lb(7) = deg2rad(2);
    model_bounds.states.x.lb(12) = deg2rad(2);
    model_bounds.states.x.ub(7) = deg2rad(88);
    model_bounds.states.x.ub(12) = deg2rad(88);
    
    model_bounds.states.x.lb(5) = 0;
    model_bounds.states.x.lb(10) = 0;
    model_bounds.states.dx.lb(5) = 0;
    model_bounds.states.dx.lb(10) = 0;
    model_bounds.states.ddx.lb(5) = 0;
    model_bounds.states.ddx.lb(10) = 0;
    model_bounds.states.x.ub(5) = 0;
    model_bounds.states.x.ub(10) = 0;
    model_bounds.states.dx.ub(5) = 0;
    model_bounds.states.dx.ub(10) = 0;
    model_bounds.states.ddx.ub(5) = 0;
    model_bounds.states.ddx.ub(10) = 0;
    
    model_bounds.states.x.lb(3) = deg2rad(1);
    model_bounds.states.x.ub(3) = deg2rad(20);
    

    
    % parameters in function: virtual_constraints
    % enforce Virtual Constraints
    model_bounds.options.enforceVirtualConstraints = true;  % if true, use curve; otherwise, use points   
    % feedback control gain for virtual constraints
    model_bounds.gains.kp = 100;
    model_bounds.gains.kd = 20;

    
    % 4-bar joint constraint wrench
    model_bounds.inputs.ConstraintWrench.ffourBar.lb = -1000*ones(6,1);     % -109
    model_bounds.inputs.ConstraintWrench.ffourBar.ub = 1000*ones(6,1);      % 109
    
    % 4-bar joint constraints
    model_bounds.params.pfourBar.lb = zeros(6,1);
    model_bounds.params.pfourBar.ub = zeros(6,1);
    model_bounds.params.pfourBar.x0 = zeros(6,1);
    
    % knee constraint wrench, 4-Bar total torques limits 
    model_bounds.constrBounds.kneetorque.lb = -130;
    model_bounds.constrBounds.kneetorque.ub = 130;
    
    
    %%% Step Duration
    model_bounds.time.duration.lb = 1*T_;
    model_bounds.time.duration.ub = T_;   % 2*T_
    model_bounds.time.duration.x0 = T_;
    
    model_bounds.time.t0.lb = 0;
    model_bounds.time.t0.ub = 0;
    model_bounds.time.t0.x0 = 0;
    
    model_bounds.time.tf.lb = 1*T_;
    model_bounds.time.tf.ub = T_;     % 2*T_
    model_bounds.time.tf.x0 = T_;
    
%     % foot_clearance (height)
%     model_bounds.constrBounds.foot_clearance.lb = 0.1;
%     model_bounds.constrBounds.foot_clearance.ub = 0.5;     

    % foot_clearance (height), separate to four parts
    model_bounds.constrBounds.foot_clearance_1.lb = 0.01;   % 0.005; 0.03 
    model_bounds.constrBounds.foot_clearance_1.ub = 0.3;
    
    model_bounds.constrBounds.foot_clearance_2.lb = 0.03;
    model_bounds.constrBounds.foot_clearance_2.ub = 0.3;   % 0.30 
    model_bounds.constrBounds.foot_clearance_2.x0 = 0.05;
    
    model_bounds.constrBounds.foot_clearance_3.lb = 0.01;   % 0.005; 0.05
    model_bounds.constrBounds.foot_clearance_3.ub = 0.3;
    
    % average step velocity
    model_bounds.constrBounds.averageVelocity.lb = speed;
    model_bounds.constrBounds.averageVelocity.ub = speed;
    
%     model_bounds.constrBounds.yaw_initial.lb = 0;
%     model_bounds.constrBounds.yaw_initial.ub = 0;  

    % start swing foot velocity, pos vel
    % (Make sure foot goes upward)
    model_bounds.constrBounds.footVelocityBeginning.lb = [ -5, 0, 0]';
    model_bounds.constrBounds.footVelocityBeginning.ub = [ 5, 0, 10]';
    % impact swing foot velocity, pos vel
    % (Make sure foot goes downward and slightly backward)    
    model_bounds.constrBounds.footVelocityEnd.lb = [ -5, 0, -10]';
    model_bounds.constrBounds.footVelocityEnd.ub = [ 5, 0, -0]';
    

    %%% Common Virtual Constraints
    % bezier coefficient for virtual constraints "position",
    % degree = 5, 5+1 parameters, 4 curves, 6*4=24
    model_bounds.params.aposition.lb = -100*ones(6*4,1); 
    model_bounds.params.aposition.ub = 100*ones(6*4,1);
    model_bounds.params.aposition.x0 = zeros(6*4,1);
    % phase paramaters for virtual constraints "position"
    model_bounds.params.pposition.lb = [model_bounds.time.t0.lb, model_bounds.time.tf.lb]'; 
    model_bounds.params.pposition.ub = [model_bounds.time.t0.ub, model_bounds.time.tf.ub]';   
    model_bounds.params.pposition.x0 = [model_bounds.time.t0.x0, model_bounds.time.tf.x0]';
    
    
    model_bounds.params.pRightPoint.lb = [0, -wt/2.0, 0]';    
    model_bounds.params.pRightPoint.ub = [0, -wt/2.0, 0]';    
    model_bounds.params.pRightPoint.x0 = [0, -wt/2.0, 0]';
    
    model_bounds.params.pLeftPoint.lb = [0.3, wt/2.0, 0]';    
    model_bounds.params.pLeftPoint.ub = [0.4, wt/2.0, 0]';     
    model_bounds.params.pLeftPoint.x0 = [0.32, wt/2.0, 0]';

    
    %% construct the boundary values for each domain 
    % bounds has 4 fields : 
    %   RightStance
    %   LeftImpact
    %   LeftStance
    %   RightImpact
    bounds = struct();
      
    %% bounds.Right Stance
    
    bounds.RightStance = model_bounds;
    
    % RightPoint's 3 ConstraintWrench (x y z)ï¼?
    % 2 Tangential friction, 1 Vertical support force
    bounds.RightStance.inputs.ConstraintWrench.fRightPoint.lb = [-10000,-10000, 0]';
    bounds.RightStance.inputs.ConstraintWrench.fRightPoint.ub = [10000,10000,10000]';
    %bounds.RightStance.inputs.ConstraintWrench.fRightPoint.x0 = [100,100,260]';
    
    % constraints for step length and step width
    %bounds.RightStance.constrBounds.stepWidth.lb = wt - T_*vy;
    %bounds.RightStance.constrBounds.stepWidth.ub = wt - T_*vy;
    bounds.RightStance.constrBounds.stepWidth.lb = wt;
    bounds.RightStance.constrBounds.stepWidth.ub = wt;
    bounds.RightStance.constrBounds.stepLength.lb = -T_*vx ;  % -T_*vx + 0.01
    bounds.RightStance.constrBounds.stepLength.ub = -T_*vx ;  % -T_*vx - 0.01
    % make sure the x-drict clearance at mid of period --jun, 20180514
    bounds.RightStance.constrBounds.stepLength_mid.lb = -0.05 ;   
    bounds.RightStance.constrBounds.stepLength_mid.ub = 0.05 ; 
    bounds.RightStance.constrBounds.stepLength_mid.x0 = 0.0 ; 

    
    %% bounds.LeftImpact
    
    % just copy states bounds, not reset/change
    bounds.LeftImpact.states.x = model_bounds.states.x;
    bounds.LeftImpact.states.xn = model_bounds.states.x;
    bounds.LeftImpact.states.dx = model_bounds.states.dx;
    bounds.LeftImpact.states.dxn = model_bounds.states.dx;
    
    bounds.LeftImpact.inputs = struct();
    
    bounds.LeftImpact.inputs.ConstraintWrench.ffourBar.lb = -1000*ones(6,1);    % 109
    bounds.LeftImpact.inputs.ConstraintWrench.ffourBar.ub = 1000*ones(6,1);
    
    bounds.LeftImpact.inputs.ConstraintWrench.fLeftPoint.lb = [-10000,-10000, 0]';
    bounds.LeftImpact.inputs.ConstraintWrench.fLeftPoint.ub = [10000,10000,10000]';

    bounds.LeftImpact.params = struct();


    %% bounds.Right Stance
    
    bounds.LeftStance = model_bounds;

    % LeftPoint's 3 ConstraintWrench (x y z)ï¼? 
    % 2 Tangential friction, 1 Vertical support force
    bounds.LeftStance.inputs.ConstraintWrench.fLeftPoint.lb = [-10000,-10000, 0]';
    bounds.LeftStance.inputs.ConstraintWrench.fLeftPoint.ub = [10000,10000,10000]';
    
    % constraints for step length and step width
    %bounds.LeftStance.constrBounds.stepWidth.lb = wt + T_*vy;
    %bounds.LeftStance.constrBounds.stepWidth.ub = wt + T_*vy;
    bounds.LeftStance.constrBounds.stepWidth.lb = wt;
    bounds.LeftStance.constrBounds.stepWidth.ub = wt;
    bounds.LeftStance.constrBounds.stepLength.lb = T_*vx ;    % T_*vx - 0.01
    bounds.LeftStance.constrBounds.stepLength.ub = T_*vx ;    % T_*vx + 0.01
    % make sure the x clearance at mid of period --jun, 20180514
    bounds.LeftStance.constrBounds.stepLength_mid.lb = -0.05 ;   
    bounds.LeftStance.constrBounds.stepLength_mid.ub = 0.05;
    bounds.LeftStance.constrBounds.stepLength_mid.x0 = 0.0 ; 
    
    
    %% bounds.RightImpact
    
    % just copy states bounds, not reset/change
    bounds.RightImpact.states.x = model_bounds.states.x;
    bounds.RightImpact.states.xn = model_bounds.states.x;
    bounds.RightImpact.states.dx = model_bounds.states.dx;
    bounds.RightImpact.states.dxn = model_bounds.states.dx;

    bounds.RightImpact.inputs = struct();
    
    bounds.RightImpact.inputs.ConstraintWrench.ffourBar.lb = -1000*ones(6,1);   % 109
    bounds.RightImpact.inputs.ConstraintWrench.ffourBar.ub = 1000*ones(6,1);
    
    bounds.RightImpact.inputs.ConstraintWrench.fRightPoint.lb = [-10000,-10000, 0]';
    bounds.RightImpact.inputs.ConstraintWrench.fRightPoint.ub = [10000,10000,10000]';
    
    bounds.RightImpact.params = struct();

  
end
