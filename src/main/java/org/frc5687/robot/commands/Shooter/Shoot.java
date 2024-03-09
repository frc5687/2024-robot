package org.frc5687.robot.commands.Shooter;

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
    private final Shooter _shooter;
    private final Intake _intake;
    private final DriveTrain _driveTrain;
    private final RobotState _robotState = RobotState.getInstance();

    public Shoot(
        Shooter shooter,
        Intake intake,
        DriveTrain driveTrain
    ) {
        _shooter = shooter;
        _intake = intake;
        _driveTrain = driveTrain;
        addRequirements(_shooter, _intake);
    }


    @Override
    public void initialize() {
        super.initialize();
        _shooter.setConfigSlot(0);
    }

    @Override
    public void execute() {
        // Pair<Double, Double> shooterRPMAndAngle = _robotState.calculateAdjustedRPMAndAngleToTarget();
        // _shooter.setShooterMotorRPM(shooterRPMAndAngle.getFirst().doubleValue());
        // Rotation2d angle = new Rotation2d(shooterRPMAndAngle.getSecond() + Math.PI); // FIXME HACKING IN FOR TESTING DO NOT DOE
        // _driveTrain.setSnapHeading(angle);

        Pair<Double, Double> distanceAndAngle = _robotState.getDistanceAndAngleToSpeaker();
        double distance = distanceAndAngle.getFirst();
        // add max distance conditional?
        _shooter.setRPMFromDistance(distance);
        Rotation2d angle = new Rotation2d(distanceAndAngle.getSecond());
        _driveTrain.goToHeading(angle);

        boolean isInAngle = Math.abs(_driveTrain.getHeading().minus(angle).getRadians()) < Constants.DriveTrain.TARGET_TOLERANCE;
        boolean isAtTargetRPM = _shooter.isAtTargetRPM();
        metric("IsInAngle", isInAngle);
        metric("isAtTargetRPM", isAtTargetRPM);
        
        if (isAtTargetRPM && isInAngle) { 
            _intake.setSpeed(Constants.Intake.INTAKE_SPEED);
        }

        SmartDashboard.putNumber("Angle to shoot", angle.getRadians());
        SmartDashboard.putNumber("Angle Error", Math.abs(_driveTrain.getHeading().minus(angle).getRadians()));
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
