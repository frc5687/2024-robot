package org.frc5687.robot.commands.Shooter;

import org.frc5687.robot.Constants;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.Intake;
import org.frc5687.robot.subsystems.Shooter;

public class Shoot extends OutliersCommand{
    private Shooter _shooter;

    private Intake _intake;

    public Shoot(Shooter shooter,
    Intake intake) {
        _shooter = shooter;
        _intake = intake;
        addRequirements(_shooter, _intake);
    }

    public void execute() {
        _shooter.setTargetRPM(Constants.Shooter.SHOOT_RPM);
        _shooter.setToTarget();
        if (_shooter.isAtTargetRPM()) { 
            _intake.setSpeed(Constants.Intake.INTAKE_SPEED);
        }
    }

    public boolean isFinished(boolean interrupted) {
        return false;
    }
}
