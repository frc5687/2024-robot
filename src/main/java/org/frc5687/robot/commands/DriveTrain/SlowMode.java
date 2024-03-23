package org.frc5687.robot.commands.DriveTrain;

import static org.frc5687.robot.Constants.DriveTrain.HIGH_KINEMATIC_LIMITS;

import org.frc5687.lib.swerve.SwerveSetpointGenerator.KinematicLimits;
import org.frc5687.robot.Constants;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.DriveTrain;

public class SlowMode extends OutliersCommand {
    private final DriveTrain _driveTrain;

    public SlowMode(DriveTrain driveTrain) {
        _driveTrain = driveTrain;
    }

    @Override
    public void initialize() {
        _driveTrain.disableAutoShifter();
    }

    @Override
    public void execute() {
        _driveTrain.setKinematicLimits(Constants.DriveTrain.SLOW_MODE_KINEMATIC_LIMITS);
        if (!_driveTrain.isLowGear()) {
            _driveTrain.shiftDownModules();
        }
    }
    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        _driveTrain.enableAutoShifter();
        KinematicLimits limits = (_driveTrain.isLowGear() ? Constants.DriveTrain.LOW_KINEMATIC_LIMITS : Constants.DriveTrain.HIGH_KINEMATIC_LIMITS);
        _driveTrain.setKinematicLimits(limits);
    }
}
