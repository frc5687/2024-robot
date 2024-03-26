package org.frc5687.robot.commands.DriveTrain;

import edu.wpi.first.math.geometry.Rotation2d;

import org.frc5687.lib.control.SwerveHeadingController.HeadingState;
import org.frc5687.robot.Constants;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.DriveTrain;

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
        _driveTrain.setHeadingControllerState(HeadingState.ON);
        _driveTrain.goToHeading(
            _driveTrain.isRedAlliance() ? _rotation.minus(new Rotation2d(Math.PI)) : _rotation);
    }

    @Override
    public boolean isFinished() {
        return _driveTrain.getHeading().minus(_rotation).getRadians() < Constants.DriveTrain.HEADING_TOLERANCE;
    }

    @Override
    public void end(boolean interrupted) {
        error(" finished");
        if(interrupted)
        {
            metric("Snap to was interupted", interrupted);
        }
        super.end(interrupted);
    }
}

