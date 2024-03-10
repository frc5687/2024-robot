package org.frc5687.robot.commands.DriveTrain;

import edu.wpi.first.math.geometry.Rotation2d;

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
        _driveTrain.goToHeading(
            _driveTrain.isRedAlliance() ? _rotation.minus(new Rotation2d(Math.PI)) : _rotation);
    }

    @Override
    public boolean isFinished() {
        return _driveTrain.getHeading().minus(_rotation).getRadians() < Constants.DriveTrain.SNAP_TOLERANCE;
    }

    @Override
    public void end(boolean interrupted) {
        error(" finished");
        super.end(interrupted);
    }
}

