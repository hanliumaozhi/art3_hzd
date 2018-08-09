function robot = LoadModel(urdf, load_path, delay_set)
    
    if nargin < 2
        load_path = [];
    end
    
    if nargin < 1
        cur = utils.get_root_path();
        urdf = fullfile(cur,'urdf','art3_description.urdf');
    end
    
    if nargin < 3
        delay_set = false;
    end
    
    base = get_base_dofs('planar'); % for Art3-2D: planar ; Art3-3D : floating 
            
    limits = [base.Limit];
    [limits.lower] = deal(-5, 0.7,-deg2rad(5));
    [limits.upper] = deal(5, 1.3, deg2rad(50));
    [limits.velocity] = deal(2, 2, 7.65);
    [limits.effort] = deal(0);
    for i=1:3
        base(i).Limit = limits(i);
    end
    robot = Art3Model(urdf, base, load_path);
    
    
    if isempty(load_path)
        configureDynamics(robot, 'DelayCoriolisSet', delay_set);
%         configureDynamics(robot, 'DelayCoriolisSet', delay_set, 'OmitCoriolisSet', true);
    else
        loadDynamics(robot, load_path, delay_set);
%         loadDynamics(robot, load_path, delay_set, 'OmitCoriolisSet', true);
    end
end

