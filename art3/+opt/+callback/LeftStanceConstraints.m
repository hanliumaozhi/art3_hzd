function LeftStanceConstraints(nlp, bounds, varargin)
    domain = nlp.Plant;
    
    ip = inputParser;
    ip.addParameter('LoadPath',[],@ischar);
    ip.parse(varargin{:});
    
    %% virtual constraints    
    opt.constraint.virtual_constraints(nlp, bounds, ip.Results.LoadPath);
    
    %% foot clearance
    [right_foot_frame] = sys.frames.RightPoint(domain);
    opt.constraint.foot_clearance(nlp, bounds, right_foot_frame);
    
    %% swing toe position
    opt.constraint.step_distance(nlp, bounds);
    
    %% swing foot velocity
    opt.constraint.impact_velocity(nlp, bounds, right_foot_frame);
    
    %% average velocity
    opt.constraint.average_velocity(nlp, bounds);
    
    %% knee torque
    opt.constraint.knee_torque(nlp, bounds);
    
    %% for better gait, looking like human, you can add other proper Constraints as you like 
    
    
end
