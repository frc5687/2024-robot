package org.frc5687.robot.commands;

import org.frc5687.robot.RobotState;

public class DisableAimAfterTime extends OutliersCommand{
    private long _timeout;

    public DisableAimAfterTime(long timeout) {
        _timeout = timeout;
    }

    @Override
    public void initialize() {
        _timeout = _timeout + System.currentTimeMillis();
        warn("started disable after");
    }

    @Override
    public boolean isFinished() {
        return System.currentTimeMillis() > _timeout;
    }

    @Override
    public void end(boolean interrupted) {
        RobotState.getInstance().setAutoAiming(false);
        warn("ended disable after");
    }
}
