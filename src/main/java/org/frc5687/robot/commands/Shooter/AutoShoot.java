package org.frc5687.robot.commands.Shooter;

import java.util.Optional;

import javax.swing.plaf.OptionPaneUI;
import javax.swing.text.html.Option;

import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotState;
import org.frc5687.robot.subsystems.DriveTrain;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.Shooter;
import org.frc5687.robot.subsystems.Intake;
import org.frc5687.robot.subsystems.Lights;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoShoot extends OutliersCommand{
    private final Shooter _shooter;
    private final Intake _intake;
    private final DriveTrain _driveTrain;
    private final Lights _lights;
    private final RobotState _robotState = RobotState.getInstance();

    private Optional<Long> _intakeTimestamp;

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
        _shooter.setConfigSlot(0);
        _intakeTimestamp = Optional.empty();
        _lights.setDebugLightsEnabled(true);
        error("Init AutoShoot");
    }

    @Override
    public void execute() {
        // Pair<Double, Double> shooterRPMAndAngle = _robotState.calculateAdjustedRPMAndAngleToTarget();
        // _shooter.setShooterMotorRPM(shooterRPMAndAngle.getFirst().doubleValue());
        // Rotation2d angle = new Rotation2d(shooterRPMAndAngle.getSecond() + Math.PI); // FIXME HACKING IN FOR TESTING DO NOT DOE
        // _driveTrain.setSnapHeading(angle);

        Pair<Double, Double> distanceAndAngle = _robotState.getDistanceAndAngleToSpeaker();
        double distance = distanceAndAngle.getFirst();
        Optional<Double> visionDistance = _robotState.getDistanceToSpeakerFromVision();
        Optional<Rotation2d> angle = _robotState.getAngleToSpeakerFromVision();
        if (visionDistance.isPresent()) {
            _shooter.setRPMFromDistance(visionDistance.get());
        } else {
            _shooter.setRPMFromDistance(distance);
        }

        ChassisSpeeds speeds = _driveTrain.getMeasuredChassisSpeeds();
        boolean isStopped = (speeds.vxMetersPerSecond < 0.1 && speeds.vyMetersPerSecond < 0.1);
        boolean isAtTargetRPM = _shooter.isAtTargetRPM();
        boolean isInAngle = _robotState.isAimedAtSpeaker();
        SmartDashboard.putBoolean("Shoot/IsInAngle", isInAngle);
        SmartDashboard.putBoolean("Shoot/isAtTargetRPM", isAtTargetRPM);
        SmartDashboard.putBoolean("Shoot/isStopped", isStopped);

        if (angle.isPresent()) {
            Rotation2d visionHeading = _driveTrain.getHeading().minus(angle.get());
            metric("VisionAngleYaw", angle.get().getRadians());
            metric("VisionAngleHeading", visionHeading.getRadians());
            _driveTrain.goToHeading(visionHeading);
        } else {
            _driveTrain.goToHeading(new Rotation2d(distanceAndAngle.getSecond()));
        }
        
        _driveTrain.setVelocity(new ChassisSpeeds(0.0, 0.0, _driveTrain.getRotationCorrection()));
        
        if (isAtTargetRPM && isInAngle && isStopped) { 
            if (_intakeTimestamp.isEmpty()) {
                _intakeTimestamp = Optional.of(System.currentTimeMillis());
            }
            _intake.setSpeed(Constants.Intake.INTAKE_SPEED);
            metric("Shot RPM of: ", _shooter.getTargetRPM());
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
        _lights.setDebugLightsEnabled(false);
    }
}
