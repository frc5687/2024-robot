package org.frc5687.robot.commands.Shooter;

import org.frc5687.lib.control.SwerveHeadingController.HeadingState;
import org.frc5687.robot.Constants;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.Shooter;
import org.frc5687.robot.subsystems.Deflector;
import org.frc5687.robot.subsystems.Intake;
import org.frc5687.robot.subsystems.DriveTrain;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Shoot extends OutliersCommand{
    private Shooter _shooter;
    private Deflector _deflector;
    private Intake _intake;
    private DriveTrain _driveTrain;
    private Pose3d _tagPose;

    public Shoot(
        Shooter shooter,
        Deflector deflector,
        Intake intake,
        DriveTrain driveTrain
    ) {
        _shooter = shooter;
        _deflector = deflector;
        _intake = intake;
        _driveTrain = driveTrain;
        addRequirements(_shooter, _intake, _deflector);
    }

    @Override
    public void initialize() {
        _tagPose = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField().getTagPose(
            DriverStation.getAlliance().get() == Alliance.Red ? 4 : 7
        ).get();
    }

    @Override
    public void execute() {
        Pose2d robotPose = _driveTrain.getEstimatedPose();

        double xDistance = _tagPose.getX() - robotPose.getX();
        double yDistance = _tagPose.getY() - robotPose.getY();

        double distance = Math.sqrt(
            Math.pow(xDistance, 2) + Math.pow(yDistance, 2)
        );

        double angle = Math.atan2(yDistance, xDistance) + Math.PI;

        _shooter.setTargetRPM(_shooter.calculateRPMFromDistance(distance));
        _shooter.setToTarget();
        _deflector.setTargetAngle(_deflector.calculateAngleFromDistance(distance));
        _driveTrain.setSnapHeading(new Rotation2d(angle));

        if (_shooter.isAtTargetRPM() && _deflector.isAtTargetAngle() && _driveTrain.getHeading().getRadians() - angle < Constants.DriveTrain.SNAP_TOLERANCE) { 
            _intake.setSpeed(Constants.Intake.INTAKE_SPEED);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        _driveTrain.setHeadingControllerState(HeadingState.MAINTAIN);
    }
}
