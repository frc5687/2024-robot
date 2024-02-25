package org.frc5687.robot.commands.Shooter;

import org.frc5687.lib.control.SwerveHeadingController.HeadingState;
import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotState;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.Shooter;
import org.frc5687.robot.subsystems.DriveTrain;
import org.frc5687.robot.subsystems.Intake;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shoot extends OutliersCommand{
    private Shooter _shooter;
    private Intake _intake;
    private DriveTrain _driveTrain;
    private RobotState _robotState;

    public Shoot(
        Shooter shooter,
        Intake intake,
        DriveTrain driveTrain,
        RobotState robotState
    ) {
        _shooter = shooter;
        _intake = intake;
        _driveTrain = driveTrain;
        _robotState = robotState;
        addRequirements(_shooter, _intake);
    }

    @Override
    public void execute() {
        Pair<Double, Double> distanceAndAngle = _robotState.getDistanceAndAngleToSpeaker();

        double distance = distanceAndAngle.getFirst();

        // add max distance conditional?
        double requestRPM = _shooter.calculateRPMFromDistance(distance);
        _shooter.setTargetRPM(requestRPM);
        _shooter.setToTarget();
        // Pair<Double, Double> distanceAndAngleMoving = _robotState.calculateAdjustedAngleToTarget(requestRPM);
        Rotation2d angle = new Rotation2d(distanceAndAngle.getSecond());
        _driveTrain.setSnapHeading(angle);
        boolean isInAngle = Math.abs(_driveTrain.getHeading().minus(angle).getRadians()) < Constants.DriveTrain.SNAP_TOLERANCE;
        metric("IsInAngle", isInAngle);
        if (_shooter.isAtTargetRPM() && isInAngle) { 
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
