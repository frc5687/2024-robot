package org.frc5687.robot.commands.Shooter;

import static org.frc5687.robot.Constants.DriveTrain.VISION_KINEMATIC_LIMITS;

import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotState;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.Shooter;
import org.frc5687.robot.subsystems.DriveTrain;
import org.frc5687.robot.subsystems.Intake;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class AutoShoot extends OutliersCommand{
    private Shooter _shooter;
    private Intake _intake;
    private DriveTrain _driveTrain;
    private RobotState _robotState;
    private long _endingTimestamp;

    public AutoShoot(
        Shooter shooter,
        Intake intake,
        DriveTrain driveTrain,
        RobotState robotState
    ) {
        _shooter = shooter;
        _intake = intake;
        _driveTrain = driveTrain;
        _robotState = robotState;
        addRequirements(_shooter, _intake, _driveTrain);
    }

    @Override
    public void initialize() {
        super.initialize();
        _endingTimestamp = Long.MAX_VALUE; // it will never be this big
        _driveTrain.setKinematicLimits(VISION_KINEMATIC_LIMITS);
    }

    @Override
    public void execute() {
        Pair<Double, Double> distanceAndAngle = _robotState.getDistanceAndAngleToSpeaker();

        double distance = distanceAndAngle.getFirst();
        Rotation2d angle = new Rotation2d(distanceAndAngle.getSecond());

        // add max distance conditional?
        _shooter.setTargetRPM(_shooter.calculateRPMFromDistance(distance));
        _shooter.setToTarget();
        // pid
        Rotation2d currentHeading = _driveTrain.getHeading();
        _driveTrain.setSnapHeading(angle);
        _driveTrain.setVelocity(new ChassisSpeeds(0.0, 0.0, _driveTrain.getRotationCorrection()));
        error("Desired angle: "+angle.getDegrees()+"\n Current angle: "+_driveTrain.getHeading().getDegrees());
        boolean isInAngle = Math.abs(_driveTrain.getHeading().minus(angle).getRadians()) < Constants.DriveTrain.SNAP_TOLERANCE;
        metric("IsInAngle", isInAngle);
        if (_shooter.isAtTargetRPM() && isInAngle) {
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
        _shooter.setToStop();
    }
}
