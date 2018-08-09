function knee_torque(nlp, bounds)
    % constraints for 4-Bar total torque  ----jun, 20180516

    domain = nlp.Plant;
      
    u = domain.Inputs.Control.u;
    
    ul = tovector(u(3) + u(4) - u(5));
    ul_func = SymFunction(['leftkneetorque_',domain.Name],ul,{u});
    ur = tovector(u(8) + u(9) - u(10));
    ur_func = SymFunction(['rightkneetorque_',domain.Name],ur,{u});
    
    lb = bounds.constrBounds.kneetorque.lb;
    ub = bounds.constrBounds.kneetorque.ub;
    
    addNodeConstraint(nlp, ul_func, {'u'}, 'all', lb, ub, 'NonLinear');
    addNodeConstraint(nlp, ur_func, {'u'}, 'all', lb, ub, 'NonLinear');


end