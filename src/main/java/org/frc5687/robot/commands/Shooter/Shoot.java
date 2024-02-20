package org.frc5687.robot.commands.Shooter;

import org.frc5687.lib.control.SwerveHeadingController.HeadingState;
import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotState;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.Shooter;
import org.frc5687.robot.subsystems.Deflector;
import org.frc5687.robot.subsystems.DriveTrain;
import org.frc5687.robot.subsystems.Intake;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shoot extends OutliersCommand{
    private Shooter _shooter;
    private Deflector _deflector;
    private Intake _intake;
    private DriveTrain _driveTrain;
    private RobotState _robotState;

    public Shoot(
        Shooter shooter,
        Deflector deflector,
        Intake intake,
        DriveTrain driveTrain,
        RobotState robotState
    ) {
        _shooter = shooter;
        _deflector = deflector;
        _intake = intake;
        _driveTrain = driveTrain;
        _robotState = robotState;
        addRequirements(_shooter, _intake, _deflector);
    }

    @Override
    public void execute() {
        Pair<Double, Double> distanceAndAngle = _robotState.getDistanceAndAngleToSpeaker();

        double distance = distanceAndAngle.getFirst();

        Rotation2d angle = new Rotation2d(distanceAndAngle.getSecond());

        if (distance < Constants.Shooter.MAX_DEFLECTOR_DISTANCE) {
            _shooter.setTargetRPM(Constants.Shooter.SHOOTER_RPM_WHEN_DEFLECTOR);
            _shooter.setToTarget();
            _deflector.setTargetAngle(_deflector.calculateAngleFromDistance(distance));
        } else {
            _shooter.setTargetRPM(_shooter.calculateRPMFromDistance(distance));
            _shooter.setToTarget();
            _deflector.setTargetAngle(Constants.Deflector.IDLE_ANGLE);
        }
        _driveTrain.setSnapHeading(angle);
        boolean isInAngle = Math.abs(_driveTrain.getHeading().minus(angle).getRadians()) < Constants.DriveTrain.SNAP_TOLERANCE;
        metric("IsInAngle", isInAngle);
        if (_shooter.isAtTargetRPM() && _deflector.isAtTargetAngle() && isInAngle) { 
            _intake.setSpeed(Constants.Intake.INTAKE_SPEED);
        }

        SmartDashboard.putNumber("Angle to shoot", angle.getRadians());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        _driveTrain.setMaintainHeading(_driveTrain.getHeading());
    }
}
