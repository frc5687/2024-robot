package org.frc5687.robot.commands;

import org.frc5687.robot.RobotState;

public class DisableVisionUpdates extends OutliersCommand {
    private RobotState _robotState = RobotState.getInstance();

    public DisableVisionUpdates() {}

    @Override
    public void initialize() {
        super.initialize();
        _robotState.disableVisionUpdates();
    }

    @Override
    public void end(boolean interrupted) {
        _robotState.disableVisionUpdates(); // just double check
        super.end(interrupted);
    }
}
