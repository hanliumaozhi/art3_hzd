classdef Art3Model < RobotLinks
    
   properties
        % An array of Contact Points
        %
        % @type struct
        ContactPoints 
        
        % An array of FB3 Points
        %
        % @type struct
        FB3_Points
       
   end
   
   methods
       
       function obj = Art3Model(urdf_file, base, load_path)
            % class constructor function
                      
            if nargin < 2
                load_path = [];
            end      
            
            %% customized the URDF
            % load model from the URDF file
            % normally we only need to provides the URDF file name.            
            [name, links, joints, transmissions] = ros_load_urdf(urdf_file);
            
            %% remove base_link & unused joints
            % No use, as I delete joint: 'pelvis_fixed' & link: 'base_link' manually in URDF
            % the reason why we do not use this operation is that :
            % in conflict with the 1st way to deal with 2 redundant hip_roll DOF for Art3-2D
            % see the top commentary of this m-file
            
%             % find fixed joints
             %fixed_joints = arrayfun(@(x)strcmp(x.Type,'fixed'),joints);
             %ref_joint_indices = find(fixed_joints);   
%             % remove unused joints
             %joints(ref_joint_indices) = [];
%             % remove base_link
%             base_link = arrayfun(@(x)strcmp(x.Name,'base_link'),links);
%             links(base_link)  = [];
            

            %% customized the URDF
            % re-arrange the joint orders
            all_joint_names = {joints.Name};
            disp(all_joint_names);
            nj = numel(joints);    
            indices = zeros(nj,1);
            joint_names = {
                'pelvis_fixed'
                'l_hip_roll'
                'l_hip_pitch'
                'l_knee_pitch'
                'l_FB_1'
                'l_FB_2'
                'r_hip_roll'
                'r_hip_pitch'
                'r_knee_pitch'
                'r_FB_1'
                'r_FB_2'
                };
            for i=1:nj
                index = str_index(all_joint_names,joint_names{i});
                if isempty(index)
                    warning('the joint %s not exists.', joint_names{i});
                    indices(i) = NaN;
                else
                    indices(i) = index;
                end
                
            end
            joints = joints(indices);
            
            
            %% call superclass constructor with custom configuration model
            config.name = name;
            config.joints = joints;
            config.links = links;
            config.transmissions = transmissions;
   
            obj = obj@RobotLinks(config, base, load_path);
            
            
            %% Add constraint: 4-Bar holonomic & set hip_roll fixed  
            q = obj.States.x;
    
            h = [...
                q('l_knee_pitch') - q('l_FB_1');
                q('l_knee_pitch') + q('l_FB_2');
                q('r_knee_pitch') - q('r_FB_1');
                q('r_knee_pitch') + q('r_FB_2');
                q('l_hip_roll') - 0.0000;
                q('r_hip_roll') - 0.0000;
                ];
    
            four_bar_constr = HolonomicConstraint(obj, h, 'fourBar',...
                'ConstrLabel',{{'lFB1','lFB2','rFB1','rFB2','lhiproll','rhiproll'}},...
                'DerivativeOrder',2, 'LoadPath', load_path);
    
            obj = addHolonomicConstraint(obj, four_bar_constr, load_path);
            
            %% define FB_3 frames, while unused for now ^-^
            % l_leg
            l_FB2_frame = obj.Joints(getJointIndices(obj, 'l_FB_2'));
            obj.FB3_Points.l_FB3 = CoordinateFrame(...
                'Name','l_FB3',...
                'Reference',l_FB2_frame,...
                'Offset',[0,0,-0.46],...
                'R',[0,0,0]... % z-axis is the normal axis, so no rotation required
                );
            % r_leg
            r_FB2_frame = obj.Joints(getJointIndices(obj, 'r_FB_2'));
            obj.FB3_Points.r_FB3 = CoordinateFrame(...
                'Name','r_FB3',...
                'Reference',r_FB2_frame,...
                'Offset',[0,0,-0.46],...
                'R',[0,0,0]... % z-axis is the normal axis, so no rotation required
                );
            
       end
           
   end
    
end
