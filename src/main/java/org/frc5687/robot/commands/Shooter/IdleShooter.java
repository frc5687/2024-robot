package org.frc5687.robot.commands.Shooter;

import org.frc5687.robot.Constants;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.Shooter;

public class IdleShooter extends OutliersCommand{
    public Shooter _shooter;

    public IdleShooter(Shooter shooter) {
        _shooter = shooter;
        addRequirements(_shooter);
    }

    public void execute() {
        _shooter.setIdle();
    }

    public boolean isFinished() {
        return false;
    }
}
