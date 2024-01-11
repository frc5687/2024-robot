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

    public void initialize() {
        _shooter.setSpeed(0);
    }

    public boolean isFinished(boolean interrupted) {
        return false;
    }

    public void end(boolean interrupted) {
        _shooter.setSpeed(0);
    }
}
