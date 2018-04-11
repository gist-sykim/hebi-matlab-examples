function [ group, kin, effortOffset, gravityVec ] = setupArm(kit, family)
% SETUPARM creates models for various pre-configured arm kits
%
%   [ group, kin, effortOffset, gravityVec ] = setupArm(kitName, family)
%
% The 'Kit' argument currently supports the following names:
%
%    '6dof', '5dof', '4dof', '4dof-scara'
%
% The 'Family' argument specifies the family name of the modules that
% should be selected. It is optional and defaults to 'Arm'.
%
% Note: If you are using different types of actuators or links, you will
% need to create a new custom script, or change this script accordingly.

if nargin < 2
   family = 'Arm'; 
end

%% Setup kinematic models
switch kit
    
     case 'workshop-6dof-with-gripper'
        %%
        % Create group for communications
        group = HebiLookup.newGroupFromNames(family, {
            'Base'
            'Shoulder'
            'Elbow'
            'Wrist1'
            'Wrist2'
            'Wrist3' });
        
        % Kinematic Model
        kin = HebiKinematics();
        kin.addBody('X8-3');
        kin.addBody('X5-HeavyBracket', 'mount', 'right-inside');
        kin.addBody('X8-9', 'PosLim', [-.1 1.2]); % gas spring limits
        kin.addBody('X5-Link', 'extension', 0.325, 'twist', pi);
        kin.addBody('GenericLink', 'com', zeros(3,1), 'mass', 0.3, 'output', eye(4)); % cables
        kin.addBody('X5-4');
        kin.addBody('GenericLink', 'Output', eye(4), 'mass', 0.4, 'com', [0.07 0 0.05]); % gripper-pulley
        kin.addBody('X5-Link', 'extension', 0.325, 'twist', pi);
        kin.addBody('X5-4');
        kin.addBody('X5-LightBracket', 'mount', 'right');
        kin.addBody('X5-1');
        kin.addBody('X5-LightBracket', 'mount', 'right');
        kin.addBody('X5-1');
        kin.addBody('GenericLink', 'com', zeros(3,1), 'mass', 0.2, 'output', eye(4)); % cables
        gripperOut = eye(4);
        gripperOut(3,4) = 0.1;
        kin.addBody('GenericLink', 'Output', gripperOut, 'mass', 0.2, 'com', [0 0 0.01]); % gripper
        
        % Account for external efforts due to the gas spring
        effortOffset = [0 -9.8 0 0 0 0];
        
    case '6dof' % A-2084-06
        %%
        % Create group for communications
        group = HebiLookup.newGroupFromNames(family, {
            'Base'
            'Shoulder'
            'Elbow'
            'Wrist1'
            'Wrist2'
            'Wrist3' });
        
        % Kinematic Model
        kin = HebiKinematics();
        kin.addBody('X8-9');
        kin.addBody('X5-HeavyBracket', 'mount', 'right-outside');
        kin.addBody('X8-9', 'PosLim', [-0.6 1.2]); % gas spring limits
        kin.addBody('X5-Link', 'extension', 0.325, 'twist', pi);
        kin.addBody('X5-9');
        kin.addBody('X5-Link', 'extension', 0.325, 'twist', pi);
        kin.addBody('X5-1');
        kin.addBody('X5-LightBracket', 'mount', 'right');
        kin.addBody('X5-1');
        kin.addBody('X5-LightBracket', 'mount', 'right');
        kin.addBody('X5-1');
        
        % Account for external efforts due to the gas spring
        effortOffset = [0 -9.8 0 0 0 0];
        
    case '5dof' % A-2084-05
        %%
        % Create group for communications
        group = HebiLookup.newGroupFromNames(family, {
            'Base'
            'Shoulder'
            'Elbow'
            'Wrist1'
            'Wrist2' });
        
        % Kinematic Model
        kin = HebiKinematics();
        kin.addBody('X8-9');
        kin.addBody('X5-HeavyBracket', 'mount', 'right-outside');
        kin.addBody('X8-9', 'PosLim', [-0.6 1.2]); % gas spring limits
        kin.addBody('X5-Link', 'extension', 0.325, 'twist', pi);
        kin.addBody('X5-9');
        kin.addBody('X5-Link', 'extension', 0.325, 'twist', pi);
        kin.addBody('X5-1');
        kin.addBody('X5-LightBracket', 'mount', 'right');
        kin.addBody('X5-1');
        
        % Account for external efforts due to the gas spring
        effortOffset = [0 -9.8 0 0 0];
        
    case '4dof' % A-2085-04
        %%
        % Create group for communications
        group = HebiLookup.newGroupFromNames(family, {
            'Base'
            'Shoulder'
            'Elbow'
            'Wrist1' });
        
        % Kinematic Model
        kin = HebiKinematics();
        kin.addBody('X5-4');
        kin.addBody('X5-HeavyBracket', 'mount', 'right-outside');
        kin.addBody('X5-9');
        kin.addBody('X5-Link', 'extension', 0.325, 'twist', pi);
        kin.addBody('X5-4');
        kin.addBody('X5-Link', 'extension', 0.325, 'twist', pi);
        kin.addBody('X5-1');
        
    case '4dof-scara' % A-2084-01
        %%
        % Create group for communications
        group = HebiLookup.newGroupFromNames(family, {
            'Base'
            'Shoulder'
            'Elbow'
            'Wrist1' });
        
        % Kinematic Model
        kin = HebiKinematics();
        kin.addBody('X5-4');
        kin.addBody('X5-HeavyBracket', 'mount', 'right-outside');
        kin.addBody('X5-9');
        kin.addBody('X5-Link', 'extension', 0.325, 'twist', pi/2);
        kin.addBody('X5-4');
        kin.addBody('X5-Link', 'extension', 0.325, 'twist', 0);
        kin.addBody('X5-1');
        
    otherwise
        error([kit ' is not a supported kit name']);
        
end


%% Common Setup
% Use default effort offsets
if ~exist('effortOffset', 'var') || isempty(effortOffset)
    effortOffset = zeros(1, kin.getNumDoF);
end

% Determine initial gravity vector based on the internal pose filter of
% the base module
fbk = group.getNextFeedbackFull();
baseRotMat = HebiUtils.quat2rotMat( [ 
    fbk.orientationW(1), ...
    fbk.orientationX(1), ...
    fbk.orientationY(1), ...
    fbk.orientationZ(1) ] );
gravityVec = -baseRotMat(3,1:3);

% If the base module does not report a quaternion (e.g. old firmware), then
% fall back to using the accelerometer. This assumes that the base is idle
% and that the accelerometers are reading only gravity.
if any(isnan(gravityVec))
    gravityVec = -[fbk.accelX(1) fbk.accelY(1) fbk.accelZ(1)];
    gravityVec = gravityVec / norm(gravityVec);
end

end

