clc; clear; %close all;

%% Define Manipuilator (Book)
dhparams = [0   	0	    0.1585   0;
            0	    -pi/2   0.150    0;
            0.37187  0	0           pi/2;
            0       -pi/2	0   	0;
            0       pi/2	0.417   0;
            0       -pi/2	    0   0;
            0       0       0.239   0
            ];

manipulator_uvms = rigidBodyTree;

body0 = rigidBody('body0');
jnt0 = rigidBodyJoint('jnt0','fixed');
link1 = rigidBody('link1');
jnt1 = rigidBodyJoint('jnt1','revolute');
link2 = rigidBody('link2');
jnt2 = rigidBodyJoint('jnt2','revolute');
link3 = rigidBody('link3');
jnt3 = rigidBodyJoint('jnt3','revolute');
link4 = rigidBody('link4');
jnt4 = rigidBodyJoint('jnt4','revolute');
link5 = rigidBody('link5');
jnt5 = rigidBodyJoint('jnt5','revolute');
link6 = rigidBody('link6');
jnt6 = rigidBodyJoint('jnt6','revolute');
%body7 = rigidBody('body7');
%jnt7 = rigidBodyJoint('jnt7','revolute');
%body_e = rigidBody('body_e');
%jnt_e = rigidBodyJoint('jnt_e','fixed'); 

setFixedTransform(jnt0,dhparams(1,:),'dh');
setFixedTransform(jnt1,dhparams(2,:),'dh');
setFixedTransform(jnt2,dhparams(3,:),'dh');
setFixedTransform(jnt3,dhparams(4,:),'dh');
setFixedTransform(jnt4,dhparams(5,:),'dh');
setFixedTransform(jnt5,dhparams(6,:),'dh');
setFixedTransform(jnt6,dhparams(7,:),'dh');
%setFixedTransform(jnt7,dhparams(7,:),'dh');
%setFixedTransform(jnt_e,dhparams(8,:),'dh');

body0.Joint = jnt0;
link1.Joint = jnt1;
link2.Joint = jnt2;
link3.Joint = jnt3;
link4.Joint = jnt4;
link5.Joint = jnt5;
link6.Joint = jnt6;
%body7.Joint = jnt7;
%body_e.Joint = jnt_e;

addBody(manipulator_uvms,body0,'base')
addBody(manipulator_uvms,link1,'body0')
addBody(manipulator_uvms,link2,'link1')
addBody(manipulator_uvms,link3,'link2')
addBody(manipulator_uvms,link4,'link3')
addBody(manipulator_uvms,link5,'link4')
addBody(manipulator_uvms,link6,'link5')
%addBody(manipulator_uvms,body7,'body6')
%addBody(manipulator_uvms,body_e,'body7')

showdetails(manipulator_uvms)

figure (2);
show(manipulator_uvms);


%% robot configuration (as Image) GOOOOOD!! finally :)
config = randomConfiguration(manipulator_uvms);
config(1).JointPosition = 0;
config(2).JointPosition = pi/6;
config(3).JointPosition = -pi/2-pi/6;
config(4).JointPosition = 0;
config(5).JointPosition = 0;
config(6).JointPosition = 0;

figure(3);
show(manipulator_uvms,config);
%axis([-0.5,0.5,-0.5,0.5,-0.5,0.5])
%axis off

geoJacob = geometricJacobian(manipulator_uvms,config,"link6")