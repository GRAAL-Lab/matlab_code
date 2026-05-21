%% Arm Model Class - GRAAL Lab
classdef armModel < handle
    % iTj_0 is an object containing the trasformations from the frame <i> to <i'> which
    % for q = 0 is equal to the trasformation from <i> to <i+1> = >j>
    % (see notes)
    % jointType is a vector containing the type of the i-th joint (0 rotation, 1 prismatic)
    % jointNumber is a int and correspond to the number of joints
    % q is a given configuration of the joints
    % iTj is  vector of matrices containing the transformation matrices from link i to link j for the input q.
    % The size of iTj is equal to (4,4,numberOfLinks)
    % eTt (OPTIONAL) add a tool to the model rigid attached to the
    % end-effector
    properties
        %iTj_0
        %jointType
        jointN
        %iTj
        %iTj_q
        q
        eTt
        dhparams
        arm
        config
    end

    methods
        % Constructor to initialize the geomModel property
        function self = armModel(dhparams)
            if nargin > 1
                 if ~exist('eTt','var')
                     % third parameter does not exist, so default it to something
                      eTt = eye(4);
                 end
            end
            arm = rigidBodyTree;

            body0 = rigidBody('body0');
            jnt0 = rigidBodyJoint('jnt0','fixed');
            setFixedTransform(jnt0,dhparams(1,:),'dh');
            body0.Joint = jnt0;
            addBody(arm,body0,'base');

            jointN = size(dhparams)-1;
            for i=1:jointN(1)
                link = rigidBody("link"+int2str(i));
                jnt = rigidBodyJoint("jnt"+int2str(i),'revolute');
                setFixedTransform(jnt,dhparams(i+1,:),'dh');
                link.Joint = jnt;
                if(i==1)
                    addBody(arm,link,'body0');
                else
                    addBody(arm,link,"link"+int2str(i-1))
                end
            end
        end

        function [bJe] = getJacobian(self, q)
            self.q = q;
            for i=1:size(q)
                self.config(i).JointPosition = q(i);
            end
            bJe = geometricJacobian(self.arm,self.config,"link6");

        end

        % function [bTe] = getTransform(self, q)
        %     self.q = q;
        %     bTe = getTransform(self.arm,q,"link5");
        % end
        % 
        % function [bTe] = getToolTransform(self, q)
        %     self.q = q;
        %     bTe = getTransform(self.arm,q,"link6");
        % end

    end
end