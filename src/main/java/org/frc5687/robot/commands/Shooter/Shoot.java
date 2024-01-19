package org.frc5687.robot.commands.Shooter;

import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.Shooter;

public class Shoot extends OutliersCommand{
    public Shooter _shooter;

    public Shoot(Shooter shooter) {
        _shooter = shooter;
        addRequirements(_shooter);
    }

    public void initialize() {
        _shooter.enableMotor();
    }

    public boolean isFinished(boolean interrupted) {
        return false;
    }

    public void end(boolean interrupted) {
        _shooter.disableMotor();
    }
}
