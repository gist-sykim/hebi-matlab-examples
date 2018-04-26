% -------------------------------------------------------------------------
% This demo is the same as ex_target_chase.m, with the difference being
% that the target is based on joystick input rather than the mouse
% location, and that it includes a variable tip axis.
%
% !!!! WARNING !!!!
% Before running this code we recommend changing the allowed workspace
% in the 'getTargetCoordinates' function below and setting safety limits
% on the base/elbow/shoulder joints. You may increase the speed factor,
% but we recommend starting out slow.
% -------------------------------------------------------------------------

%% Setup
% Robot specific setup. Edit setupArm.m as needed for your configuration.
[ group, kin, effortOffset, gravityVec ] = setupArm('workshop-6dof-with-gripper');
gripper = HebiLookup.newGroupFromNames('Arm', 'Gripper');
group.send('gains', HebiUtils.loadGains('ex_6dof_gains_repeat'));
 
% Trajectory
trajGen = HebiTrajectoryGenerator(kin);
trajGen.setMinDuration(0.2); % Speed up 'small' movements (default is >1s)
trajGen.setSpeedFactor(0.5); % Speed multiplier (1 = full speed, 0.5 = half)

% Connect to joystick
joy = SonyPS4Gamepad(HebiJoystick(1));

%% Continuously move to target 
fbk = group.getNextFeedbackFull();
cmd = CommandStruct();
gripperCmd = CommandStruct();

joy.axisLowpass = 0.08;

% Start background logging
group.startLog();

% Move to current coordinates
[xyzTarget, so3] = getTargetCoordinates(joy);
ikPosition = kin.getIK(...
    'xyz', xyzTarget, ...
    'so3', so3, ...
    'initial', [0 1 2.5 -1 1 1]);

traj = trajGen.newJointMove([fbk.position; ikPosition]);

endVelocities = zeros(1, group.getNumModules);
endAccels = zeros(1, group.getNumModules);

xyzLast = [nan nan nan];
so3Last = [nan nan nan]';

t0 = fbk.time;

maxDemoTime = 300; % sec
demoTimer = tic;

while toc(demoTimer) < maxDemoTime
   
    % Gather feedback
    fbk = group.getNextFeedback();
    fbkPosition = fbk.position;

    t = min(fbk.time - t0, traj.getDuration()); % bound to max duration
    
    % Get state of current trajectory
    [pos,vel,accel] = traj.getState(t);
    cmd.position = pos;
    cmd.velocity = vel;
    
    % Convert accelerations to torques
    dynamicsComp = kin.getDynamicCompEfforts( ...
        fbkPosition, pos, vel, accel );
    gravComp = kin.getGravCompEfforts( fbkPosition, gravityVec );
    
    % Add impedance control torques
    damperGains = 1 * [1; 1; 1; .0; .0; .0;]; % N or Nm / m/s
    springGains = 150 * [1; 1; 1; .0; .0; .0];  % N/m or Nm/rad
    
    J = kin.getJacobianEndEffector(fbkPosition);
    velError = J * (vel - fbk.velocity)';
    
    cmdXyz = kin.getForwardKinematicsEndEffector(pos);
    fbkXyz = kin.getForwardKinematicsEndEffector(fbkPosition);
    posError = [(cmdXyz(1:3,4) - fbkXyz(1:3,4)); zeros(3,1)];
    
    impedanceTorque = J' * ...
        (springGains .* posError + damperGains .* velError);
    
    % Sum all torques
    cmd.effort = dynamicsComp + gravComp + effortOffset + impedanceTorque';
    
    % Send current state to robot
    group.send(cmd);
    
    % Recompute trajectory if target has changed
    [xyzTarget, so3, gripperPos] = getTargetCoordinates(joy);
    if any(xyzLast ~= xyzTarget) || any(any(so3Last ~= so3))
        xyzLast = xyzTarget;
        so3Last = so3;
        
        % Find target using inverse kinematics
        ikPosition = kin.getIK( 'xyz', xyzTarget, ...
            'so3', so3, ...
            'initial', fbkPosition); % seed with current location
        
        % Start new trajectory at the current state
        t0 = fbk.time;
        traj = trajGen.newJointMove( [pos; ikPosition], ...
                        'Velocities', [vel; endVelocities], ...
                        'Accelerations', [accel; endAccels]);  
    end
    
    % open/close gripper
    gripperCmd.position = gripperPos;
    gripper.send(gripperCmd);
    
end

% Stop background logging
hLog = group.stopLogFull();

% Plot the commands and feedback
plotLogCommands(hLog, group)

%% Get target coordinates and tip axis (from joystick)
function [xyz, so3, gripperPos] = getTargetCoordinates(joy)

% Set possible workspace range [m]
world_x = [+0.15 +0.50];
world_y = [-0.25 +0.25];
world_z = [-0.20 +0.10];
angle_x = [+pi/2 -pi/2];
angle_y = [+3*pi/2 +pi/2];

% Linear mapping function (k * x + d) (note: axes are [-1, +1]
mapLinear = @(range, axis) (1-axis)/2 * diff(range) + range(1);

% Map input [0,1] to workspace [m] (linear k * x + d)
state = read(joy);
x = mapLinear(world_x, state.AXIS_LEFT_STICK_Y);
y = mapLinear(world_y, state.AXIS_LEFT_STICK_X);
z = mapLinear(world_z, (state.AXIS_LEFT_TRIGGER - state.AXIS_RIGHT_TRIGGER)/2);
xyz = [x,y,z];

% Map input [0,1] to x/y rotation [rad]
xAngle = mapLinear(angle_x, state.AXIS_RIGHT_STICK_X);
yAngle = mapLinear(angle_y, state.AXIS_RIGHT_STICK_Y);
so3 = R_y(yAngle) * R_x(xAngle);

gripperPos = -1.2 * state.BUTTON_RIGHT_TRIGGER;

end
