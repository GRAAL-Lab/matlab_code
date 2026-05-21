%% robot configuration (as Image) GOOOOOD!! finally :)
config = randomConfiguration(arm);
config(1).JointPosition = 0;
config(2).JointPosition = pi/6;
config(3).JointPosition = -pi/2-pi/6;
config(4).JointPosition = 0;
config(5).JointPosition = 0;
config(6).JointPosition = 0;

figure(1);
show(arm,config);
%axis([-0.5,0.5,-0.5,0.5,-0.5,0.5])
%axis off

geoJacob = geometricJacobian(arm,config,"link6")

%%
q      = [0, 0, 0, 0, 0, 0]';
robotModel2 = UvmsModel('Deximus');    
arm = robotModel2.arm;
arm_conf = randomConfiguration(arm);
for i=1:length(q)
    arm_conf(i).JointPosition = q(i);
end
bJe = geometricJacobian(arm, arm_conf, "link6");
