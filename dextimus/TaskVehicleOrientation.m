classdef TaskVehicleOrientation < Task   
    properties
        alignment_error;
        rpy_ref;
        rpy;
    end


    methods
        function updateReference(obj, robot)
            %v_target_distance = robot.wTgv(1:3,4) - robot.wTv(1:3,4);
            %control_ref = -0.6 * v_target_distance;
            obj.rpy_ref = robot.vehicle_rpy_ref;
            eulZYX = rotm2eul(robot.wTv(1:3,1:3));
            %pitch = eulZYX(2); roll = eulZYX(3); 

            obj.rpy = [eulZYX(3) eulZYX(2) eulZYX(1)];
            obj.alignment_error = obj.rpy_ref - obj.rpy;

            control_ref = 0.6 * obj.alignment_error;
            control_ref = Saturate(control_ref, 0.8);
            obj.xdotbar = control_ref';
        end
        function updateJacobian(obj, robot)
            eulZYX = rotm2eul(robot.wTv(1:3,1:3));
            r = eulZYX(3); p = eulZYX(2); %y = eulZYX(1);
            E_1 = [cos(p) sin(r)*sin(p) cos(r)*sin(p);
                    0     cos(r)*cos(p) -sin(r)*cos(p);
                    0     sin(r) cos(r)];
            E_1 = E_1/cos(p);
            obj.J = [zeros(3,6) zeros(3,3) E_1];
        end
        
        function updateActivation(obj, robot)
            obj.A = zeros(3);
            eulZYX = rotm2eul(robot.wTv(1:3,1:3));
            %pitch = eulZYX(2); roll = eulZYX(3); 
            obj.alignment_error = [eulZYX(3) eulZYX(2) eulZYX(1)];
            obj.A(1,1) = IncreasingBellShapedFunction(2*pi/180, 4*pi/180, 0, 1, abs(obj.alignment_error(1)));
            obj.A(2,2) = IncreasingBellShapedFunction(2*pi/180, 4*pi/180, 0, 1, abs(obj.alignment_error(2)));
            obj.A(3,3) = IncreasingBellShapedFunction(2*pi/180, 4*pi/180, 0, 1, abs(obj.alignment_error(3)));
        end

        function setRefRPY(obj, ref_rpy)
            obj.rpy_ref = ref_rpy;
        end
    end
end