package org.frc5687.robot.commands.DriveTrain;

import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.DriveTrain;

public class ShiftDown extends OutliersCommand {
    private DriveTrain _driveTrain;

    public ShiftDown(DriveTrain driveTrain) {
        _driveTrain = driveTrain;
    }

    @Override
    public void initialize() {
        _driveTrain.shiftDownModules();
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}
