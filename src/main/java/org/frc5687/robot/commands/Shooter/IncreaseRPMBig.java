package org.frc5687.robot.commands.Shooter;

import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.Shooter;

public class IncreaseRPMBig extends OutliersCommand{
    public Shooter _shooter;

    public IncreaseRPMBig(Shooter shooter) {
        _shooter = shooter;
        addRequirements(_shooter);
    }

    public void initialize() {
        _shooter.setTargetRPM(_shooter.getTargetRPM() + 100.0);
    }

    public boolean isFinished(boolean interrupted) {
        return true;
    }
}