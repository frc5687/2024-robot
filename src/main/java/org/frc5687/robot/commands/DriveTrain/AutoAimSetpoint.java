package org.frc5687.robot.commands.DriveTrain;

import java.util.Optional;

import org.frc5687.robot.RobotState;
import org.frc5687.robot.commands.DisableAimAfterTime;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.DriveTrain;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;


public class AutoAimSetpoint extends OutliersCommand {

    private DriveTrain _driveTrain;
    private RobotState _robotState = RobotState.getInstance();

    private Rotation2d _targetHeading;


    public AutoAimSetpoint(DriveTrain driveTrain) {
        _driveTrain = driveTrain;
    }

    @Override
    public void initialize() {
        Pair<Double, Double> distanceAndAngle = _robotState.getDistanceAndAngleToSpeaker();
        _targetHeading = Rotation2d.fromRadians(distanceAndAngle.getSecond());
        _robotState.setAutoAiming(true);
    }

    @Override
    public void execute() {
        Optional<Rotation2d> visionAngle = _robotState.getAngleToSpeakerFromVision();

        if (visionAngle.isPresent()) {
            double angleRadians = visionAngle.get().getRadians();
            metric("Vision Angle", angleRadians);
            _targetHeading = _driveTrain.getHeading().minus(Rotation2d.fromRadians(angleRadians));
            metric("Vision Angle Robot Heading", _targetHeading.getRadians());
        } else {
            Pair<Double, Double> distanceAndAngle = _robotState.getDistanceAndAngleToSpeaker();
            _targetHeading = Rotation2d.fromRadians(distanceAndAngle.getSecond());
            metric("Pose Angle Robot Heading", _targetHeading.getRadians());
        }

        _driveTrain.goToHeading(_targetHeading);
    }


    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        _driveTrain.goToHeading(_driveTrain.getHeading());
        (new DisableAimAfterTime(500)).schedule();
    }



    
}
