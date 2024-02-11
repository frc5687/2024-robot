package org.frc5687.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;

import org.frc5687.lib.control.SwerveHeadingController.HeadingState;
import org.frc5687.robot.Constants;
import org.frc5687.robot.subsystems.DriveTrain.DriveTrain;

public class SnapTo extends OutliersCommand {
    private DriveTrain _driveTrain;
    private Rotation2d _rotation;

    public SnapTo(DriveTrain driveTrain, Rotation2d rotation) {
        _driveTrain = driveTrain;
        _rotation = rotation;
    }

    @Override
    public void initialize() {
        super.initialize();
        _driveTrain.setHeadingControllerGoal(_rotation);
    }

    @Override
    public boolean isFinished() {
        if (_driveTrain.getHeading().minus(_rotation).getRadians()
                < Constants.DriveTrain.SNAP_TOLERANCE) {
            _driveTrain.setHeadingMaintainGoal(_rotation);
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        error(" finished");
        super.end(interrupted);
    }
}

