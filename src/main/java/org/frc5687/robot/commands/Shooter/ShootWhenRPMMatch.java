package org.frc5687.robot.commands.Shooter;

import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotState;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.Shooter;
import org.frc5687.robot.subsystems.DriveTrain;
import org.frc5687.robot.subsystems.Intake;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class ShootWhenRPMMatch extends OutliersCommand{
    private final Shooter _shooter;
    private final Intake _intake;
    private final DriveTrain _driveTrain;
    private long _endingTimestamp;
    private double _shootRPM;

    

    private final RobotState _robotState = RobotState.getInstance();

    public ShootWhenRPMMatch(
        Shooter shooter,
        Intake intake,
        double rpm,
        DriveTrain driveTrain
    ) {
        _shooter = shooter;
        _intake = intake;
        _driveTrain = driveTrain;
    }

    @Override
    public void initialize() {
        super.initialize();
        _endingTimestamp = Long.MAX_VALUE; // it will never be this big
    }

    @Override
    public void execute() {
        Pair<Double, Double> distanceAndAngle = _robotState.getDistanceAndAngleToSpeaker();
        double distance = distanceAndAngle.getFirst();
        _shooter.setShooterMotorRPM(_shootRPM);
        ChassisSpeeds speeds = _driveTrain.getMeasuredChassisSpeeds();
        boolean isAtRPMMatch = _shooter.isAtRPMMatch(distance, speeds.vxMetersPerSecond,speeds.vyMetersPerSecond);

        if (isAtRPMMatch){
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
