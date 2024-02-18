package org.frc5687.robot.commands.Shooter;

import org.frc5687.lib.control.SwerveHeadingController.HeadingState;
import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotState;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.commands.Intake.TimedIntake;
import org.frc5687.robot.subsystems.Shooter;
import org.frc5687.robot.subsystems.Deflector;
import org.frc5687.robot.subsystems.Intake;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class AutoShoot extends OutliersCommand{
    private Shooter _shooter;
    private Deflector _deflector;
    private Intake _intake;
    private RobotState _robotState;

    private boolean _done = false;
    private long _timestamp = Long.MAX_VALUE - 100000; // don't worry about it - xavier bradford

    public AutoShoot(
        Shooter shooter,
        Deflector deflector,
        Intake intake
    ) {
        _shooter = shooter;
        _deflector = deflector;
        _intake = intake;
        _robotState = RobotState.getInstance();
        addRequirements(_shooter, _intake, _deflector);
    }

    @Override
    public void initialize() {
        _shooter.flagAutoShooting(true);
    }

    @Override
    public void execute() {
        Pair<Double, Double> distanceAndAngle = _robotState.getDistanceAndAngleToSpeaker();
        error("shootinSHOOTSHOOTSHOOTg");

        double distance = distanceAndAngle.getFirst();

        double angle = distanceAndAngle.getSecond();

        if (distance < Constants.Shooter.MAX_DEFLECTOR_DISTANCE) {
            _shooter.setTargetRPM(Constants.Shooter.SHOOTER_RPM_WHEN_DEFLECTOR);
            _shooter.setToTarget();
            _deflector.setTargetAngle(_deflector.calculateAngleFromDistance(distance));
        } else {
            _shooter.setTargetRPM(_shooter.calculateRPMFromDistance(distance));
            _shooter.setToTarget();
            _deflector.setTargetAngle(Constants.Deflector.IDLE_ANGLE);
        }

        if (_shooter.isAtTargetRPM() && _deflector.isAtTargetAngle()) { 
            _intake.setSpeed(Constants.Intake.INTAKE_SPEED);
            _timestamp = System.currentTimeMillis();
        }
    }

    @Override
    public boolean isFinished() {
        return System.currentTimeMillis() > _timestamp + 1000; // 1000ms intake
    }

    @Override
    public void end(boolean interrupted) {
        // FIXME: intake might still be running at the end of autos
        _shooter.flagAutoShooting(false);
    }
}
