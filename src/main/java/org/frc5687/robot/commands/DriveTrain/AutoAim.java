package org.frc5687.robot.commands.DriveTrain;

import java.util.Optional;

import org.frc5687.robot.RobotState;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.DriveTrain;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;


public class AutoAim extends OutliersCommand {

    private DriveTrain _driveTrain;
    private RobotState _robotState = RobotState.getInstance();

    private Rotation2d _targetHeading;


    public AutoAim(DriveTrain driveTrain) {
        _driveTrain = driveTrain;
    }

    @Override
    public void initialize() {
        Pair<Double, Double> distanceAndAngle = _robotState.getDistanceAndAngleToSpeaker();
        _targetHeading = Rotation2d.fromRadians(distanceAndAngle.getSecond());
    }

    @Override
    public void execute() {
        Pair<Double, Double> distanceAndAngle = _robotState.getDistanceAndAngleToSpeaker();
        // Optional<Rotation2d> visionAngle = _robotState.getAngleToSpeakerFromVision();
        // if (visionAngle.isPresent()) {
            // _targetHeading = visionAngle.get();
            // metric("Vison angel", visionAngle.get().getRadians());
        // } else {
            // Pair<Double, Double> distanceAndAngle = _robotState.getDistanceAndAngleToSpeaker();
            // _targetHeading = Rotation2d.fromRadians(distanceAndAngle.getSecond());
        // }
        _targetHeading = Rotation2d.fromRadians(distanceAndAngle.getSecond());
        _driveTrain.goToHeading(_targetHeading);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        _driveTrain.goToHeading(_driveTrain.getHeading());
    }



    
}
