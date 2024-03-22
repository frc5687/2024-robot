package org.frc5687.robot.commands.Shooter;

import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.DriveTrain;
import org.frc5687.robot.subsystems.Intake;
import org.frc5687.robot.subsystems.Shooter;
import org.frc5687.robot.Constants;
import org.frc5687.robot.Robot;
import org.frc5687.robot.RobotState;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;

public class MovingShoot extends OutliersCommand {
    private final Shooter _shooter;
    private final Intake _intake;
    private final DriveTrain _driveTrain;
    private final RobotState _robotState;

    public MovingShoot(
        Shooter shooter, 
        Intake intake, 
        DriveTrain drivetrain,
        RobotState robotstate
    ) {
        _shooter = shooter;
        _intake = intake;
        _driveTrain = drivetrain;
        _robotState = robotstate;
    }

    @Override
    public void execute() {
        Pair<Double,Rotation2d> shooterRPMAndAngle = _robotState.calculateAdjustedRPMAndAngleToTarget();
        _shooter.setShooterMotorRPM(shooterRPMAndAngle.getFirst().doubleValue());
        _driveTrain.goToHeading(_driveTrain.getHeading().minus(shooterRPMAndAngle.getSecond()));
        _intake.setSpeed((Constants.Intake.INTAKE_SPEED));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
    @Override
    public void end(boolean interrupted) {
        _intake.setSpeed(0);
    }
}
