%% Setup
group = HebiLookup.newGroupFromNames('Arm','Wrist3');

%% Virtual Spring
stiffness = 1; % Nm/rad
cmd = CommandStruct();
t0 = tic();
while toc(t0) < 30
    
    fbk = group.getNextFeedback();
    cmd.effort = -stiffness * fbk.position;
    group.send(cmd);
    
end

%% IMU Stabilization
cmd = CommandStruct();
t0 = tic();
while toc(t0) < 30
    
    fbk = group.getNextFeedback();
    cmd.velocity = -fbk.gyroZ;
    group.send(cmd);
    
end