package org.frc5687.robot.commands.Shooter;

import java.util.Optional;

import javax.swing.plaf.OptionPaneUI;

import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotState;
import org.frc5687.robot.subsystems.DriveTrain;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.Shooter;
import org.frc5687.robot.subsystems.Intake;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class AutoShoot extends OutliersCommand{
    private final Shooter _shooter;
    private final Intake _intake;
    private final DriveTrain _driveTrain;
    private final RobotState _robotState = RobotState.getInstance();

    private Optional<Long> _intakeTimestamp;

    public AutoShoot(
        Shooter shooter,
        Intake intake,
        DriveTrain driveTrain
    ) {
        _shooter = shooter;
        _intake = intake;
        _driveTrain = driveTrain;
        addRequirements(_shooter, _intake, _driveTrain);
    }


    @Override
    public void initialize() {
        super.initialize();
        _shooter.setConfigSlot(0);
        _intakeTimestamp = Optional.empty();
    }

    @Override
    public void execute() {
        // Pair<Double, Double> shooterRPMAndAngle = _robotState.calculateAdjustedRPMAndAngleToTarget();
        // _shooter.setShooterMotorRPM(shooterRPMAndAngle.getFirst().doubleValue());
        // Rotation2d angle = new Rotation2d(shooterRPMAndAngle.getSecond() + Math.PI); // FIXME HACKING IN FOR TESTING DO NOT DOE
        // _driveTrain.setSnapHeading(angle);

        Pair<Double, Double> distanceAndAngle = _robotState.getDistanceAndAngleToSpeaker();
        double distance = distanceAndAngle.getFirst();
        // Optional<Double> visionDistance = _robotState.getDistanceToSpeakerFromVision();
        Optional<Rotation2d> angle = _robotState.calculateAngleToTagFromVision(_robotState.getSpeakerTargetTagId());
        // if (visionDistance.isPresent()) {
            // _shooter.setRPMFromDistance(visionDistance.get());
        // } else {
            _shooter.setRPMFromDistance(distance);
        // }

        ChassisSpeeds speeds = _driveTrain.getMeasuredChassisSpeeds();
        boolean isStopped = (speeds.vxMetersPerSecond < 0.1 && speeds.vyMetersPerSecond < 0.1);

        if (angle.isPresent()) {
            _driveTrain.goToHeading(_driveTrain.getHeading().minus(angle.get()));
        } else {
            _driveTrain.goToHeading(new Rotation2d(distanceAndAngle.getSecond()));
        }

        
        _driveTrain.setVelocity(new ChassisSpeeds(0.0, 0.0, _driveTrain.getRotationCorrection()));

        boolean isAtTargetRPM = _shooter.isAtTargetRPM();
        boolean isInAngle = _robotState.isAimedAtSpeaker();
        
        if (isAtTargetRPM && isInAngle && isStopped) { 
            if (_intakeTimestamp.isEmpty()) {
                _intakeTimestamp = Optional.of(System.currentTimeMillis());
            }
            _intake.setSpeed(Constants.Intake.INTAKE_SPEED);
        }
    }

    @Override
    public boolean isFinished() {
        if (_intakeTimestamp.isPresent()) {
            return System.currentTimeMillis() > _intakeTimestamp.get() + 150; // 200ms intake
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        _intake.setSpeed(0.0);
    }
}
