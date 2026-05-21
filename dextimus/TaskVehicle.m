classdef TaskVehicle < Task   
    properties

    end


    methods
        function updateReference(obj, robot)
            v_target_distance = robot.wTgv(1:3,4) - robot.wTv(1:3,4);
            control_ref = -0.6 * v_target_distance;
            control_ref = Saturate(control_ref, 0.8);
            obj.xdotbar = control_ref;
        end
        function updateJacobian(obj, robot)
            obj.J = [zeros(3,6) -robot.wTv(1:3,1:3) zeros(3)];
        end
        
        function updateActivation(obj, robot)
            obj.A = eye(3);
        end
    end
end