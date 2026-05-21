classdef UvmsModel < handle
    %UVMSMODEL Vehicle–Manipulator System Kinematic Model
    %
    % This class represents the kinematic state and geometry of an
    % Underwater Vehicle–Manipulator System (UVMS).
    % It provides:
    %   - storage of joint and vehicle states
    %   - forward kinematics computation
    %   - Jacobian computation
    %   - goal configuration definition
    %
    % The class does not implement any control or simulation logic.
    %
    % Example:
    %   model = UvmsModel('Robust');
    %   model.updateKinematics();

    properties
        %% --- State variables ---
        q           % [6x1] manipulator joint positions
        q_dot       % [6x1] manipulator joint velocities
        eta         % [6x1] vehicle pose [x y z r p y]
        v_nu        % [6x1] vehicle velocites (linear and angular) proj. on the vehicle frame
        altitude
                

        %% --- Geometry ---
        vTb         % fixed transform from vehicle to manipulator base
        eTt         % transform from end-effector to tool point

        arm         % robotic arm 
        arm_conf    % arm configuration = q

        %% --- Limits ---
        jlmin       % joint lower limits
        jlmax       % joint upper limits

        %% --- Transformations ---
        wTv         % vehicle wrt world
        vTw         % world wrt vehicle
        vTe         % end-effector wrt vehicle
        vTt         % tool wrt vehicle
        wTt         % tool wrt world
        wTg         % goal (tool) wrt wolrd
        wTgv        % goal (vehicle) wrt world
        bTe         % end-effector wrt arm base
        bTt         % tool wrt arm base
        vTg         % goal (tool) wrt vehicle

        %% --- Goals ---
        wRg         % desired orientation of tool in world
        goalPosition% desired position of tool in world
        wRgv        % desired vehicle orientation in world
        vehicleGoalPosition % desired vehicle position in world

        %% ---Reference Values ---
        vehicle_pos_ref
        vehicle_rpy_ref
        arm_pos_ref
        arm_rpy_ref

    end

    methods
        function obj = UvmsModel(robotType)
            % Constructor
            %   robotType - string: 'DexROV' or 'Robust'
            %
            % Initializes geometry, default state, and goal transforms.

            if nargin < 1
                robotType = 'Robust';
            end

            % Define the geometry of the manipulator mounting
            switch robotType
                case 'DexROV'
                    obj.vTb = [eul2rotm([pi, 0, pi]) [0.167; 0; -0.43]; 0 0 0 1];
                case 'Dextimus'
                    obj.vTb = [eul2rotm([-pi/2, pi, 0]) [0.167; 0; -0.43]; 0 0 0 1]; % [z,y,x] order!!!
                    %obj.vTb = [rotation(pi, 0, -pi/2) [0.167; 0; -0.43]; 0 0 0 1];
                otherwise % 'Robust'
                    obj.vTb = [rotation(0, 0, pi) [0.85; 0; -0.42]; 0 0 0 1];
            end

            % Initialize default state
            obj.q      = [0.0 -0.6 0 0 0.6 0]';%[0, 0, 0, 0, 0, 0]';
            obj.eta    = [11.0 38.0 -37 0.3 -0.3 0.5]'; %[8.5 37.5 -38 0 -0.06 0.5]' [11.5, 38.5, -37]'

            % Default limits
            obj.jlmin  = [-pi, -pi/2, -pi/2, -pi, -pi/2, -pi]';
            obj.jlmax  = [pi, pi/2, pi/2, pi, pi/2, pi]';

            % Initialize transformations
            obj.wTv = eye(4);
            obj.vTw = eye(4);
            obj.vTe = eye(4);
            obj.vTt = eye(4);
            obj.wTt = eye(4);
            %obj.eTt = eye(4); % tool coincides with end-effector by default

            eRt = eye(3);
            e_r_te = [0, 0, 0.23924]';
            obj.eTt = [eRt e_r_te; 0 0 0 1];

            % Initialize goal placeholders
            obj.wTg = eye(4);
            obj.wTgv = eye(4);

            % Initialize geometric model % juri
            initializeArm(obj);
            updateTransformations(obj);


        end

        % function setGoal(obj, toolPosition, toolOrientation, vehiclePosition, vehicleOrientation)
        %     % Set goal positions and orientations for tool and vehicle
        %     %
        %     % Inputs:
        %     %   toolPosition      [3x1] desired tool position in world
        %     %   toolOrientation   [3x1] desired RPY angles (rad)
        %     %   vehiclePosition   [3x1] desired vehicle position in world
        %     %   vehicleOrientation[3x1] desired RPY angles (rad)
        % 
        %     obj.wRg = rotation(toolOrientation(1), toolOrientation(2), toolOrientation(3));
        %     obj.goalPosition = toolPosition;
        %     obj.wTg = [obj.wRg obj.goalPosition; 0 0 0 1];
        % 
        %     obj.wRgv = rotation(vehicleOrientation(1), vehicleOrientation(2), vehicleOrientation(3));
        %     obj.vehicleGoalPosition = vehiclePosition;
        %     obj.wTgv = [obj.wRgv obj.vehicleGoalPosition; 0 0 0 1];
        % end

        function setArmGoal(obj, toolPosition, toolOrientation)
            obj.wRg = rotation(toolOrientation(1), toolOrientation(2), toolOrientation(3));
            obj.goalPosition = toolPosition;
            obj.wTg = [obj.wRg obj.goalPosition; 0 0 0 1];
            obj.arm_pos_ref = toolPosition;
            obj.arm_rpy_ref = toolOrientation;
        end

        function setVehicleGoal(obj,vehiclePosition, vehicleOrientation)
            obj.wRgv = rotation(vehicleOrientation(1), vehicleOrientation(2), vehicleOrientation(3));
            obj.vehicleGoalPosition = vehiclePosition;
            obj.wTgv = [obj.wRgv obj.vehicleGoalPosition; 0 0 0 1];
        end

        function setVehicleRef(obj,vehiclePosition, vehicleOrientation)
            obj.vehicle_pos_ref = vehiclePosition;
            obj.vehicle_rpy_ref = vehicleOrientation;
        end

        function updateTransformations(obj)
            % Compute forward kinematics of the UVMS
            %
            % Updates all transformations from current state.
            obj.wTv = [rotation(obj.eta(4), obj.eta(5), obj.eta(6)) obj.eta(1:3); 0 0 0 1];
            obj.vTw = inv(obj.wTv);
            obj.vTg = obj.vTw * obj.wTg;

            %obj.bTe = RobustEndEffectorTransform(obj.q); % Enrico
            %obj.arm.updateDirectGeometry(obj.q); % before using robotic
            %tool
            %obj.bTe = obj.arm.getTransform(obj.q);
            
            obj.bTe = obj.getArmTransform(obj.q);
            obj.vTe = obj.vTb * obj.bTe;
            obj.vTt = obj.vTe * obj.eTt;

            obj.bTt = obj.getToolTransform(obj.q);
            obj.vTt = obj.vTb * obj.bTt;
            obj.wTt = obj.wTv * obj.vTt;
        end

        function initializeArm(obj)
            %obj.bTe = BuildTree();
            %jointType = [0 0 0 0 0 0];
            %obj.arm =  geometricModel(obj.bTe,jointType,obj.eTt);
            % dhparams = [0   	0	    0.1585   0;
            %             0	    -pi/2   0.150    0;
            %             0.37187  0	0           pi/2;
            %             0       -pi/2	0   	0;
            %             0       pi/2	0.417   0;
            %             0       -pi/2	    0   0;
            %             0       0       0.239   0
            %             ];
            %obj.arm = armModel(dhparams);
            obj.arm = importrobot("arm.urdf");
            % obj.arm = rigidBodyTree;
            % 
            % body0 = rigidBody('body0');
            % jnt0 = rigidBodyJoint('jnt0','fixed');
            % setFixedTransform(jnt0,dhparams(1,:),'dh');
            % body0.Joint = jnt0;
            % addBody(obj.arm,body0,'base');
            % 
            % jointN = size(dhparams)-1;
            % for i=1:jointN(1)
            %     link = rigidBody("link"+int2str(i));
            %     jnt = rigidBodyJoint("jnt"+int2str(i),'revolute');
            %     setFixedTransform(jnt,dhparams(i+1,:),'dh');
            %     link.Joint = jnt;
            %     if(i==1)
            %         addBody(obj.arm,link,'body0');
            %     else
            %         addBody(obj.arm,link,"link"+int2str(i-1))
            %     end
            % end

            obj.arm_conf = randomConfiguration(obj.arm);
        end

        function [bJe] = getArmJacobian(obj, q)
            obj.q = q;
            for i=1:length(q)
                obj.arm_conf(i).JointPosition = q(i);
            end
            bJe = geometricJacobian(obj.arm, obj.arm_conf, "link_7");

        end

        function [bTe] = getArmTransform(obj, q)
            obj.q = q;
            for i=1:length(q)
                obj.arm_conf(i).JointPosition = q(i);
            end
            obj.q = q;
            bTe = getTransform(obj.arm, obj.arm_conf, "link_6");
        end

        function [bTe] = getToolTransform(obj, q)
            obj.q = q;
            for i=1:length(q)
                obj.arm_conf(i).JointPosition = q(i);
            end
            bTe = getTransform(obj.arm, obj.arm_conf, "link_7");
        end

    end
end