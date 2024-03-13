package org.frc5687.robot.commands.Shooter;

import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotState;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.Shooter;
import org.frc5687.robot.subsystems.DriveTrain;
import org.frc5687.robot.subsystems.Intake;
import org.frc5687.robot.subsystems.Lights;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class DefinedRPMShoot extends OutliersCommand{
    private final Shooter _shooter;
    private final Intake _intake;
    private long _endingTimestamp;
    private double _shootRPM;

    public DefinedRPMShoot(
        Shooter shooter,
        Intake intake,
        double rpm
    ) {
        _shooter = shooter;
        _intake = intake;
        _shootRPM = rpm;
        addRequirements(_shooter, _intake);
    }

    @Override
    public void initialize() {
        super.initialize();
        _endingTimestamp = Long.MAX_VALUE; // it will never be this big
    }

    @Override
    public void execute() {
        _shooter.setShooterMotorRPM(_shootRPM);
        boolean isAtTargetRPM = _shooter.isAtTargetRPM();
        metric("isAtTargetRPM", isAtTargetRPM);

        if (isAtTargetRPM) {
            // trigger intake only once.... it has been triggered already if it is not MAX_VALUE O-O
            if (_endingTimestamp == Long.MAX_VALUE) {
                _intake.setSpeed(Constants.Intake.INTAKE_SPEED);
                _endingTimestamp = System.currentTimeMillis() + 250; // 100ms intake
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
        _shooter.setToStop();
    }
}
