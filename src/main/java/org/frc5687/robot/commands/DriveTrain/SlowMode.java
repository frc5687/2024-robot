package org.frc5687.robot.commands.DriveTrain;

import static org.frc5687.robot.Constants.DriveTrain.SLOW_KINEMATIC_LIMITS;

import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.DriveTrain;

public class SlowMode extends OutliersCommand {
        private DriveTrain _driveTrain;
    
    public SlowMode(DriveTrain drivetrain) {
        _driveTrain = drivetrain;
    }

    @Override
    public void execute() {
        _driveTrain.setKinematicLimits(SLOW_KINEMATIC_LIMITS);
        if (!_driveTrain.isLowGear()) {
            _driveTrain.shiftDownModules();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
