package org.frc5687.robot.commands.Shooter;

import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.Shooter;

public class IncreaseRPM extends OutliersCommand{
    public Shooter _shooter;

    public IncreaseRPM(Shooter shooter) {
        _shooter = shooter;
        addRequirements(_shooter);
    }

    public void initialize() {
        _shooter.setTargetRPM(_shooter.getTargetRPM() + 10.0);
    }

    public boolean isFinished(boolean interrupted) {
        return true;
    }
}