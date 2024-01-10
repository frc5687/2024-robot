package org.frc5687.robot.commands.Shooter;

import org.frc5687.robot.Constants;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.Shooter;

public class Shoot extends OutliersCommand{
    public Shooter _shooter;

    public Shoot(Shooter shooter) {
        _shooter = shooter;
    }

    public void initialize() {
        _shooter.setSpeed(Constants.Shooter.SHOOT_SPEED);
    }

    public boolean isFinished(boolean interrupted) {
        return false;
    }

    public void end(boolean interrupted) {
        _shooter.setSpeed(0);
    }
}
