package org.frc5687.robot.commands.DriveTrain;

import java.util.Optional;

import org.frc5687.robot.RobotState;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.DriveTrain;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;


public class PassAimSetpoint extends OutliersCommand {

    private DriveTrain _driveTrain;
    private RobotState _robotState = RobotState.getInstance();

    private Rotation2d _targetHeading;


    public PassAimSetpoint(DriveTrain driveTrain) {
        _driveTrain = driveTrain;
    }

    @Override
    public void initialize() {
        Pair<Double, Double> distanceAndAngle = _robotState.getDistanceAndAngleToCorner();
        _targetHeading = Rotation2d.fromRadians(distanceAndAngle.getSecond());
        _robotState.setAutoAiming(true);
    }

    @Override
    public void execute() {

            Pair<Double, Double> distanceAndAngle = _robotState.getDistanceAndAngleToCorner();
            _targetHeading = Rotation2d.fromRadians(distanceAndAngle.getSecond());
            metric("Corner Pose Angle Robot Heading", _targetHeading.getRadians());

        _driveTrain.goToHeading(_targetHeading);
    }


    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        _driveTrain.goToHeading(_driveTrain.getHeading());
        _robotState.setAutoAiming(false);
    }
}
