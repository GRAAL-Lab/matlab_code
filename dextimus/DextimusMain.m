clc; clear; close all;

% Add paths
%addpath('./simulation_scripts');
addpath('../tools');
addpath('../icat');
addpath('../tpik');

%%
% Simulation parameters
dt       = 0.005;
endTime  = 5;

% Initialize robot model and simulator
robotModel = UvmsModel('Dextimus');          
sim = UvmsSim(dt, robotModel, endTime);

% Initialize Unity interface
unity = UnityInterface("127.0.0.1");

% Define tasks
%task_vehicle = TaskVehicle();       
task_tool    = TaskTool();
task_vehicle = TaskVehicle();
task_vehicle_ori =TaskVehicleOrientation();
%task_vehicle_ori.setRefRPY([pi/4 0 1]);

task_H_alignment = TaskHorizontalAttitude();
task_set1 = {task_vehicle_ori, task_vehicle};
task_set2 = {task_H_alignment, task_tool };
task_set3 = {task_vehicle_ori, task_vehicle, task_tool};
task_set4 = {task_H_alignment};

% Define actions and add to ActionManagerss
actionManager = ActionManager();
%actionManager.addAction(task_set2);  % action 1
actionManager.addAction(task_set3);  % action 1

% Define desired positions and orientations (world frame)
w_arm_goal_position = [11.5, 38.5, -38.0]'; % x13 y37.3748 z-39.8860aa
w_arm_goal_orientation = [0, 0, pi/2];
w_vehicle_goal_position = [11.5, 38.0, -37]'; % [10.5, 37.5, -38]';
w_vehicle_goal_orientation = [0, 0, 0];

robotModel.setVehicleRef(w_vehicle_goal_position, w_vehicle_goal_orientation);


% Set arm goal
robotModel.setArmGoal(w_arm_goal_position, w_arm_goal_orientation);
% Set disturbance on vehicle goal
amp_dist = 0.5; T_dist = 1.0; f_dist = 1/T_dist;

% Initialize the logger
logger = SimulationLogger(ceil(endTime/dt), robotModel, task_set1, dt);


% Main simulation loop
for step = 1:sim.maxSteps
    % 1. Receive altitude from Unity
    robotModel.altitude = unity.receiveAltitude(robotModel);
    
    % 2. Compute control commands for current action
    [v_nu, q_dot] = actionManager.computeICAT(robotModel);

    % 3. Step the simulator (integrate velocities)
    sim.step(v_nu, q_dot);

    % 4. Send updated state to Unity
    unity.send(robotModel);

    % 5. Logging
    logger.update(sim.time, sim.loopCounter);

    % 1.2 Send vehicle pose (sin)
    w_vehicle_goal_position_disturbed = w_vehicle_goal_position;
    w_vehicle_goal_position_disturbed(3) = w_vehicle_goal_position_disturbed(3) + amp_dist*sin(logger.t(step)/T_dist);
    robotModel.setVehicleGoal(w_vehicle_goal_position_disturbed, w_vehicle_goal_orientation);

    % 6. Optional debug prints
    if mod(sim.loopCounter, round(1 / sim.dt)) == 0
        %fprintf('t = %.2f s\n', sim.time);
        %fprintf('alt = %.2f m\n', robotModel.altitude);
    end
    % 7. Optional real-time slowdown
    %SlowdownToRealtime(dt);

    % 8. show arm
    %figure (10);
    %show(robotModel.arm, robotModel.arm_conf, 'PreservePlot', false);
    %drawnow limitrate;
end

% Display plots
logger.plotAll();

% Print max values
logger.computeMax();
%fprintf('qdot_max = %.2f rad/s\n', logger.q_dot_max);
%fprintf('qddot_max = %.2f rad/s^2\n', logger.q_ddot_max);

% Print details
fprintf('dist_amp = %.2f m\n', amp_dist);
fprintf('dist_freq = %.2f Hz\n', f_dist);
fprintf('arm/base distance = %.2f m\n', robotModel.vehicle_pos_ref - robotModel.arm_pos_ref);

joints_vel = logger.q_dot_max
joints_acc = logger.q_ddot_max
EE_pos_error = logger.arm_error_pos
EE_ori_error = logger.arm_error_ori


% Clean up Unity interface
delete(unity);