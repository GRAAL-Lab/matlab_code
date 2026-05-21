classdef SimulationLogger < handle
    properties
        t            % time vector
        q            % joint positions
        q_dot        % joint velocities
        q_ddot       % joint accelerations
        eta          % vehicle pose
        v_nu         % vehicle velocity
        a            % task activations (diagonal only)
        xdotbar_task % reference velocities for tasks (cell array)
        robot        % robot model
        task_set     % set of tasks
        p_tool_b    % position of manipulator with respect to the base
        p_tool_w    % position of manipulator with respect to the world
        %p_tool_ref  % arm's reference goal
        rpy_tool    % orientation of manipulator with respect to the world
        %rpy_tool_ref % arm's reference rpy
        rpy_vehicle % orientation of vehicle with respect to the world
        %rpy_vehicle_ref % vehicles's reference rpy
        vehicle_ref % vehicles's reference goal
        maxL        % max loop
        q_dot_max   % max values of joints velocities
        q_ddot_max  % max values of joints accelerations
        dt          % simulation time step
        arm_error_pos
        arm_error_ori
        k % starting point to compute max values

    end

    methods
        function obj = SimulationLogger(maxLoops, robotModel, task_set, dt)
            obj.robot = robotModel;
            obj.task_set = task_set;
            obj.maxL = maxLoops;
            obj.t = zeros(1, maxLoops);
            obj.q = zeros(6, maxLoops); % 6 dof
            obj.q_dot = zeros(6, maxLoops); % 6 dof
            obj.q_ddot = zeros(6, maxLoops-1); % 6 dof
            obj.eta = zeros(6, maxLoops);
            obj.v_nu = zeros(6, maxLoops);

            obj.p_tool_b = zeros(3, maxLoops);
            obj.p_tool_w = zeros(3, maxLoops);
            obj.rpy_tool = zeros(3, maxLoops);
            obj.rpy_vehicle = zeros(3, maxLoops);

            obj.q_dot_max = zeros(6,1);
            obj.q_ddot_max = zeros(6,1);

            obj.dt = dt;
            obj.k = floor(obj.maxL/2);

            % Store the diagonal of each activation matrix
            maxDiagSize = max(cellfun(@(t) size(t.A,1), task_set));
            obj.a = zeros(maxDiagSize, maxLoops, length(task_set));

            % Initialize cell array to store task reference velocities
            obj.xdotbar_task = cell(length(task_set), maxLoops);
        end

        function update(obj, t, loop)
            % Store robot state
            obj.t(loop) = t;
            obj.q(:, loop) = obj.robot.q;
            obj.q_dot(:, loop) = obj.robot.q_dot;
            obj.eta(:, loop) = obj.robot.eta;
            obj.v_nu(:, loop) = obj.robot.v_nu;

            % Store task activations (diagonal only) and reference velocities
            for i = 1:length(obj.task_set)
                diagA = diag(obj.task_set{i}.A);           % extract diagonal
                obj.a(1:length(diagA), loop, i) = diagA;
                obj.xdotbar_task{i, loop} = obj.task_set{i}.xdotbar;
            end

            % Manipulator position
            obj.p_tool_w(:, loop) = obj.robot.wTt(1:3,4);
            % Manipulator orientation
            rotm = tform2rotm(obj.robot.wTt);
            % Returns [Yaw, Pitch, Roll] in radians
            angle_rad = rotm2eul(rotm, 'ZYX')'; 
            temp_rad = angle_rad(1);
            angle_rad(1) = angle_rad(3);
            angle_rad(3) = temp_rad;
            obj.rpy_tool(:, loop) = rad2deg(angle_rad);
            obj.rpy_vehicle(:, loop) = rad2deg(obj.eta(4:6, loop));
            
        end

        function plotAll(obj)
            % Example plotting for robot state
            figure(1);
            subplot(2,1,1);
            plot(obj.t, obj.q, 'LineWidth', 2);
            grid on
            title('Joints Angles');
            legend('q_1','q_2','q_3','q_4','q_5','q_6');
            xlabel('t [s]'); ylabel('angle [rad]');
            subplot(2,1,2);
            plot(obj.t, obj.q_dot, 'LineWidth', 2);
            title('Joints Velocities');
            legend('qd_1','qd_2','qd_3','qd_4','qd_5','qd_6');
            xlabel('t [s]'); ylabel('angular velocity [rad/s]');
            grid on

            figure(2);
            subplot(2,1,1);
            y = obj.robot.vehicle_pos_ref(1)*ones(1,obj.maxL) - obj.eta(1,:);
            plot(obj.t, y, 'LineWidth', 2);
            grid on
            hold on
            y = obj.robot.vehicle_pos_ref(2)*ones(1,obj.maxL) - obj.eta(2,:);
            plot(obj.t,y, 'LineWidth', 2);
            y = obj.robot.vehicle_pos_ref(3)*ones(1,obj.maxL) - obj.eta(3,:);
            plot(obj.t, y, 'LineWidth', 2);
            legend('x','y','z');
            title('vehicle disturbance');
            xlabel('t [s]'); ylabel('distance [m]');

            %figure (3);
            subplot(2,1,2);
            y = obj.robot.arm_pos_ref(1)*ones(1,obj.maxL) - obj.p_tool_w(1,:);
            obj.arm_error_pos(1) = max(abs(y(1,obj.k:end)));
            plot(obj.t, y, 'LineWidth', 2);
            grid on
            hold on
            y = obj.robot.arm_pos_ref(2)*ones(1,obj.maxL) - obj.p_tool_w(2,:);
            obj.arm_error_pos(2) = max(abs(y(1,obj.k:end)));
            plot(obj.t, y, 'LineWidth', 2);
            y = obj.robot.arm_pos_ref(3)*ones(1,obj.maxL) - obj.p_tool_w(3,:);
            obj.arm_error_pos(3) = max(abs(y(1,obj.k:end)));
            plot(obj.t, y, 'LineWidth', 2);
            legend('x','y','z');
            title('end-effector pos error');
            xlabel('t [s]'); ylabel('distance [m]');

            figure (4);
            subplot(2,1,1);
            z = rad2deg(obj.robot.vehicle_rpy_ref(1))*ones(1,obj.maxL) - obj.rpy_vehicle(1,:);
            plot(obj.t, z, 'LineWidth', 2);
            hold on
            z = rad2deg(obj.robot.vehicle_rpy_ref(2))*ones(1,obj.maxL) - obj.rpy_vehicle(2,:);
            plot(obj.t, z, 'LineWidth', 2);
            z = rad2deg(obj.robot.vehicle_rpy_ref(3))*ones(1,obj.maxL) - obj.rpy_vehicle(3,:);
            plot(obj.t, z, 'LineWidth', 2);
            legend('roll','pitch','yaw');
            title('Vehicle RPY');
            xlabel('t[s]'); ylabel('error [deg]'); grid on

            subplot(2,1,2);
            z = rad2deg(obj.robot.arm_rpy_ref(1))*ones(1,obj.maxL) - obj.rpy_tool(1,:);
            obj.arm_error_ori(1) = max(abs(z(1,obj.k:end)));
            plot(obj.t, z, 'LineWidth', 2);
            hold on
            z = rad2deg(obj.robot.arm_rpy_ref(2))*ones(1,obj.maxL) - obj.rpy_tool(2,:);
            obj.arm_error_ori(2) = max(abs(z(1,obj.k:end)));
            plot(obj.t, z, 'LineWidth', 2);
            z = rad2deg(obj.robot.arm_rpy_ref(3))*ones(1,obj.maxL) - obj.rpy_tool(3,:);
            obj.arm_error_ori(3) = max(abs(z(1,obj.k:end)));
            plot(obj.t, z, 'LineWidth', 2);
            legend('roll','pitch','yaw');
            title('Manipulator RPY');
            xlabel('t [s]'); ylabel('error [deg]'); grid on

            % subplot(3,1,1);
            % plot(obj.t, obj.eta(1,:), 'LineWidth', 2);
            % grid on
            % hold on
            % plot(obj.t, obj.p_tool_w(1,:),  'LineWidth', 2);
            % legend('vehicle','end-effector');
            % xlabel('t'); ylabel('X'); grid on
            % 
            % subplot(3,1,2);
            % plot(obj.t, obj.eta(2,:), 'LineWidth', 2);
            % hold on
            % plot(obj.t, obj.p_tool_w(2,:),  'LineWidth', 2);
            % legend('vehicle','end-effector');
            % xlabel('t'); ylabel('Y'); grid on
            % 
            % subplot(3,1,3);
            % plot(obj.t, obj.eta(3,:), 'LineWidth', 2);
            % hold on
            % plot(obj.t, obj.p_tool_w(3,:),  'LineWidth', 2);
            % legend('vehicle','end-effector');
            % xlabel('t'); ylabel('Z'); grid on
            % 
            figure(5);
            subplot(2,1,1);
            plot(obj.t, obj.rpy_vehicle, 'LineWidth', 2);
            legend('roll','pitch','yaw');
            title('Vehicle RPY');
            xlabel('t'); ylabel('deg'); grid on

            subplot(2,1,2);
            plot(obj.t, obj.rpy_tool, 'LineWidth', 2);
            legend('roll','pitch','yaw');
            title('Manipulator RPY');
            xlabel('t'); ylabel('deg'); grid on
        end

        function computeMax(obj)
            
            for i = 1:6
                obj.q_dot_max(i) = max(abs(obj.q_dot(i,obj.k:end)));
                obj.q_ddot(i,:) = diff(obj.q_dot(i,:))/obj.dt;
                obj.q_ddot_max(i) = max(abs(obj.q_ddot(i,obj.k:end)));
            end
        end
    end
end