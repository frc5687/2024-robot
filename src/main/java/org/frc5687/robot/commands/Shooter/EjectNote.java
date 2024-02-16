package org.frc5687.robot.commands.Shooter;

import org.frc5687.robot.Constants;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.Shooter;
import org.frc5687.robot.subsystems.Intake;

public class EjectNote extends OutliersCommand{
    private Shooter _shooter;
    private Intake _intake;

    public EjectNote(
        Shooter shooter,
        Intake intake
    ) {
        _shooter = shooter;
        _intake = intake;
        addRequirements(_shooter, _intake);
    }

    @Override
    public void execute() {
        _shooter.setTargetRPM(400);
        _shooter.setToTarget();

        if (_shooter.isAtTargetRPM()) { 
            _intake.setSpeed(Constants.Intake.INTAKE_SPEED);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
