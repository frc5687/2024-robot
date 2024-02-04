package org.frc5687.robot.commands.Shooter;

import org.frc5687.lib.control.SwerveHeadingController.HeadingState;
import org.frc5687.robot.Constants;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.DriveTrain;
import org.frc5687.robot.subsystems.Intake;
import org.frc5687.robot.subsystems.Shooter;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Shoot extends OutliersCommand{
    private Shooter _shooter;
    private DriveTrain _driveTrain;
    private Intake _intake;
    private Pose3d _tagPose;

    public Shoot(
        Shooter shooter,
        DriveTrain driveTrain,
        Intake intake
    ) {
        _shooter = shooter;
        _driveTrain = driveTrain;
        _intake = intake;
        addRequirements(_shooter, _intake);
    }

    public void initialize() {
        _tagPose = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField().getTagPose(
            DriverStation.getAlliance().get() == Alliance.Red ? 4 : 7
        ).get();
    }

    public void execute() {
        Pose2d robotPose = _driveTrain.getEstimatedPose();

        double xDistance = _tagPose.getX() - robotPose.getX();
        double yDistance = _tagPose.getY() - robotPose.getY();

        double distance = Math.sqrt(
            Math.pow(xDistance, 2) + Math.pow(yDistance, 2)
        );

        double angle = Math.atan2(yDistance, xDistance);

        _shooter.setTargetRPM(_shooter.calculateRPMFromDistance(distance));
        _shooter.setToTarget();
        _driveTrain.setSnapHeading(new Rotation2d(angle));

        if (_shooter.isAtTargetRPM() && _driveTrain.getHeading().getRadians() - angle < Constants.DriveTrain.SNAP_TOLERANCE) { 
            _intake.setSpeed(Constants.Intake.INTAKE_SPEED);
        }
    }

    public boolean isFinished(boolean interrupted) {
        return false;
    }

    public void end() {
        _driveTrain.setHeadingControllerState(HeadingState.MAINTAIN);
    }
}
