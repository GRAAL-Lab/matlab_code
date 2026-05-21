robot = importrobot("arm.urdf");
figure(10);
show(robot)
showdetails(robot)

q = randomConfiguration(robot);

show(robot,q);

for i=1:length(q)
    q(i).JointPosition = 0;
end
show(robot,q)
%%
ik = inverseKinematics(RigidBodyTree=robot);
ee = robot.BodyNames{end};
poseTarget = se3([0 0 pi/2],"eul","ZYX",[0 -0.5 1.0]);
weights = [1 1 1 0.8 0.8 0.8];
%initGuessConfig = [pi/2 0 0 0 0 0];

show(robot,q);
axis([-0.5 0.5 -1.0 0.5 -0.1 1.2])
hold on
plotTransforms(poseTarget,FrameSize=0.2);
title("Initial Guess Configuration and Pose Target")

[config,solninfo] = ik(ee,tform(poseTarget),weights,q);
show(robot,config,PreservePlot=false);
title("End-Effector Target Pose Achieved")
hold off