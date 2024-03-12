package org.frc5687.robot.commands.Shooter;

import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotState;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.Shooter;
import org.frc5687.robot.subsystems.DriveTrain;
import org.frc5687.robot.subsystems.Intake;
import org.frc5687.robot.subsystems.Lights;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class AutoShoot extends OutliersCommand{
    private final Shooter _shooter;
    private final Intake _intake;
    private final DriveTrain _driveTrain;
    private final RobotState _robotState = RobotState.getInstance();
    private final Lights _lights;
    private long _endingTimestamp;

    public AutoShoot(
        Shooter shooter,
        Intake intake,
        DriveTrain driveTrain,
        Lights lights
    ) {
        _shooter = shooter;
        _intake = intake;
        _driveTrain = driveTrain;
        _lights = lights;
        addRequirements(_shooter, _intake, _driveTrain);
    }

    @Override
    public void initialize() {
        super.initialize();
        _endingTimestamp = Long.MAX_VALUE; // it will never be this big
        _lights.setDebugLightsEnabled(true);
    }

    @Override
    public void execute() {
        Pair<Double, Double> distanceAndAngle = _robotState.getDistanceAndAngleToSpeaker();

        double distance = distanceAndAngle.getFirst();
        Rotation2d angle = new Rotation2d(distanceAndAngle.getSecond());

        // add max distance conditional?
        _shooter.setRPMFromDistance(distance);
        
        _driveTrain.goToHeading(angle);
        _driveTrain.setVelocity(new ChassisSpeeds(0.0, 0.0, _driveTrain.getRotationCorrection()));

        boolean isInAngle = _driveTrain.isHeadingInTolerance(angle, Constants.DriveTrain.SNAP_TOLERANCE);
        boolean isAtTargetRPM = _shooter.isAtTargetRPM();
        metric("IsInAngle", isInAngle);
        metric("isAtTargetRPM", isAtTargetRPM);
        _lights.setDebugValues(isInAngle, isAtTargetRPM);

        if (isAtTargetRPM && isInAngle) {
            // trigger intake only once.... it has been triggered already if it is not MAX_VALUE O-O
            if (_endingTimestamp == Long.MAX_VALUE) {
                _intake.setSpeed(Constants.Intake.INTAKE_SPEED);
                _endingTimestamp = System.currentTimeMillis() + 250; // 250ms intake
            }
        }
    }

    @Override
    public boolean isFinished() {
        return System.currentTimeMillis() > _endingTimestamp;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        _driveTrain.disableHeadingController();
        _lights.setDebugLightsEnabled(false);
        _shooter.setToStop();
    }
}
