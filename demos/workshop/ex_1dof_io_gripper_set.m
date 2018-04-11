%%
gripper = HebiLookup.newGroupFromNames('Arm', 'Gripper');
ioBoard = HebiLookup.newGroupFromNames('External', 'IO_Board');

%%
cmd = CommandStruct();
analogRange = [0.02743 4.947];
while true
   
    fbk = ioBoard.getNextFeedbackIO();
    potentiometer = (fbk.a1 - analogRange(1)) / diff(analogRange);
    button = fbk.b1;
    combined = min(1, potentiometer + button);
    cmd.position = -1.3 * combined + 0.1;
    gripper.send(cmd);
    
end