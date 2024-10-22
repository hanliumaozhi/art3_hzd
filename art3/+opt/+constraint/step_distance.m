    function step_distance(nlp, bounds)
    % constraints for step length and step width
    
    domain = nlp.Plant;
    x = domain.States.x;
    
    [right_foot_frame] = sys.frames.RightPoint(domain);
    [left_foot_frame] = sys.frames.LeftPoint(domain);
    right_foot = getCartesianPosition(domain, right_foot_frame);  
    left_foot = getCartesianPosition(domain, left_foot_frame);
    constraint = tomatrix(left_foot(1:2) - right_foot(1:2));
    constraint_func = SymFunction(['step_distance_',domain.Name], constraint, {x});
    lb = [bounds.constrBounds.stepLength.lb
        bounds.constrBounds.stepWidth.lb];
    ub = [bounds.constrBounds.stepLength.ub
        bounds.constrBounds.stepWidth.ub];
    addNodeConstraint(nlp, constraint_func, {'x'}, 'first', lb, ub, 'NonLinear');
    
    % make sure the mid_step -----by jun 20180514
    lb = [bounds.constrBounds.stepLength_mid.lb
        bounds.constrBounds.stepWidth.lb];
    ub = [bounds.constrBounds.stepLength_mid.ub
        bounds.constrBounds.stepWidth.ub];
    addNodeConstraint(nlp, constraint_func, {'x'}, floor(nlp.NumNode/2)+1, lb, ub, 'NonLinear');
    
end

