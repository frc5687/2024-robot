package org.frc5687.robot.commands.Shooter;

import java.util.Optional;

import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotState;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.Shooter;
import org.frc5687.robot.subsystems.Intake;
import org.frc5687.robot.subsystems.Lights;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class Shoot extends OutliersCommand{
    private final Shooter _shooter;
    private final Intake _intake;
    private final Lights _lights;
    private final RobotState _robotState = RobotState.getInstance();

    public Shoot(
        Shooter shooter,
        Intake intake,
        Lights lights
    ) {
        _shooter = shooter;
        _intake = intake;
        _lights = lights;
        addRequirements(_shooter, _intake);
    }


    @Override
    public void initialize() {
        super.initialize();
        _shooter.setConfigSlot(0);
        _lights.setDebugLightsEnabled(true);
    }

    @Override
    public void execute() {
        // Pair<Double, Double> shooterRPMAndAngle = _robotState.calculateAdjustedRPMAndAngleToTarget();
        // _shooter.setShooterMotorRPM(shooterRPMAndAngle.getFirst().doubleValue());
        // Rotation2d angle = new Rotation2d(shooterRPMAndAngle.getSecond() + Math.PI); // FIXME HACKING IN FOR TESTING DO NOT DOE
        // _driveTrain.setSnapHeading(angle);
        ChassisSpeeds speeds = _robotState.getMeasuredSpeeds();
        boolean isStopped = (Math.abs(speeds.vxMetersPerSecond) < 0.1 && Math.abs(speeds.vyMetersPerSecond) < 0.1 && Math.abs(speeds.omegaRadiansPerSecond) < 0.1);

        Pair<Double, Double> distanceAndAngle = _robotState.getDistanceAndAngleToSpeaker();
        double distance = distanceAndAngle.getFirst();
        // Optional<Double> visionDistance = _robotState.getDistanceToSpeakerFromVision();
        // if (visionDistance.isPresent()) {
            // _shooter.setRPMFromDistance(visionDistance.get());
        // } else {
            _shooter.setRPMFromDistance(distance);
        // }

        boolean isAtTargetRPM = _shooter.isAtTargetRPM();
        boolean isInAngle = _robotState.isAimedAtSpeaker();
        metric("IsInAngle", isInAngle);
        metric("isAtTargetRPM", isAtTargetRPM);
        
        if (isAtTargetRPM && isInAngle && isStopped) { 
            _intake.setSpeed(Constants.Intake.INTAKE_SPEED);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        _lights.setDebugLightsEnabled(false);
    }
}
