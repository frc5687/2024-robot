package org.frc5687.robot.commands;

import org.frc5687.robot.RobotState;

public class EnableVisionUpdates extends OutliersCommand {
    private RobotState _robotState = RobotState.getInstance();

    public EnableVisionUpdates() {}

    @Override
    public void initialize() {
        super.initialize();
        _robotState.enableVisionUpdates();
    }

    @Override
    public void end(boolean interrupted) {
        _robotState.enableVisionUpdates(); // just double check
        super.end(interrupted);
    }
}
