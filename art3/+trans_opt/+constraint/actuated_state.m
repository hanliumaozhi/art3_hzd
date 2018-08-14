function actuated_state(nlp, node, bounds)
    % constraints for step length and step width
    
    domain = nlp.Plant;
    x = domain.States.x;
    dx = domain.States.dx;
    idx = [6:7, 11:12]; 
    full_state = [x(idx);dx(idx)];
    % knee angle
    full_state_fun = SymFunction(['actState_',domain.Name], full_state, {x, dx});
    addNodeConstraint(nlp, full_state_fun, {'x', 'dx'}, node, ...
        bounds.constrBounds.actuated_state.lb, ...
        bounds.constrBounds.actuated_state.ub,'Linear');
    
end

