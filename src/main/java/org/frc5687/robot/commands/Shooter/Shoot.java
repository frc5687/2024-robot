package org.frc5687.robot.commands.Shooter;

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
        _shooter.enableMotor();
        // 1200 is current rpm
        if (Math.abs(_shooter.getMotorRPM() - 1200) < 50) { 
            _intake.setSpeed(1.0);
        }
    }

    public boolean isFinished(boolean interrupted) {
        return false;
    }

    public void end(boolean interrupted) {
        _shooter.disableMotor();
    }
}
