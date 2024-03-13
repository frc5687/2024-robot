package org.frc5687.robot.commands.Shooter;

import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotState;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.Shooter;
import org.frc5687.robot.subsystems.Intake;

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
        // _shooter.setManualShootRPM(3200);
        _shooter.setToManualShoot();

        if (_shooter.isAtTargetRPM()) {
            _intake.setSpeed(Constants.Intake.INTAKE_SPEED);
        }
      
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
