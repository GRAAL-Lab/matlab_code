classdef TaskHorizontalAttitude < Task   
    properties
        alignment_error;
    end


    methods
        function updateReference(obj, robot)
            %v_target_distance = robot.wTgv(1:3,4) - robot.wTv(1:3,4);
            %control_ref = -0.6 * v_target_distance;
            eulZYX = rotm2eul(robot.wTv(1:3,1:3));
            %pitch = eulZYX(2); roll = eulZYX(3); 

            obj.alignment_error = [eulZYX(3) eulZYX(2) 0];
            control_ref = 0.6 * obj.alignment_error;
            control_ref = Saturate(control_ref, 0.8);
            obj.xdotbar = control_ref';
        end
        function updateJacobian(obj, robot)
            obj.J = [zeros(3,6) zeros(3,3) -robot.wTv(1:3,1:3)];
        end
        
        function updateActivation(obj, robot)
            obj.A = zeros(3);
            eulZYX = rotm2eul(robot.wTv(1:3,1:3));
            %pitch = eulZYX(2); roll = eulZYX(3); 
            obj.alignment_error = [eulZYX(3) eulZYX(2)];
            obj.A(1,1) =  IncreasingBellShapedFunction(3*pi/180, 5*pi/180, 0, 1, abs(obj.alignment_error(1)));
            obj.A(2,2) =  IncreasingBellShapedFunction(4*pi/180, 8*pi/180, 0, 1, abs(obj.alignment_error(2)));
            obj.A(3,3) = 0;
        end
    end
end