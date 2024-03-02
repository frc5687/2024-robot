package org.frc5687.robot.commands.Shooter;

import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotState;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.Shooter;
import org.frc5687.robot.subsystems.DriveTrain;
import org.frc5687.robot.subsystems.Intake;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ManualShoot extends OutliersCommand{
    private Shooter _shooter;
    private Intake _intake;
    private RobotState _robotState = RobotState.getInstance();

    public ManualShoot(
        Shooter shooter,
        Intake intake
    ) {
        _shooter = shooter;
        _intake = intake;
        addRequirements(_shooter, _intake);
    }

    @Override
    public void execute() {
        Pair<Double, Double> distanceAndAngle = _robotState.getDistanceAndAngleToSpeaker();

        double distance = distanceAndAngle.getFirst();

        double angle = distanceAndAngle.getSecond();

        _shooter.setTargetRPM(2700);
        _shooter.setToTarget();

        // if (_shooter.getBottomMotorRPM() > 2600 && _shooter.getBottomMotorRPM() < 2800) { 
        if (_shooter.isAtTargetRPM()) {
            _intake.setSpeed(Constants.Intake.INTAKE_SPEED);
        }

        // SmartDashboard.putNumber("Angle to shoot", angle);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // _driveTrain.setMaintainHeading(_driveTrain.getHeading());
    }
}
