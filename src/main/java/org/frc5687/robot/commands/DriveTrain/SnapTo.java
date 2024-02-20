package org.frc5687.robot.commands.DriveTrain;

import edu.wpi.first.math.geometry.Rotation2d;

import org.frc5687.lib.control.SwerveHeadingController.HeadingState;
import org.frc5687.robot.Constants;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.DriveTrain;
import org.frc5687.robot.subsystems.DriveTrain.ControlState;

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
        _driveTrain.setControlState(ControlState.MANUAL);
        _driveTrain.setSnapHeading(_rotation);
    }

    @Override
    public boolean isFinished() {
        if (_driveTrain.getHeading().minus(_rotation).getRadians() < Constants.DriveTrain.SNAP_TOLERANCE) {
            _driveTrain.setHeadingControllerState(HeadingState.MAINTAIN);
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

