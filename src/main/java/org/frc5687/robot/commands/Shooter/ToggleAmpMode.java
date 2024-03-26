package org.frc5687.robot.commands.Shooter;

import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.Shooter;

public class ToggleAmpMode extends OutliersCommand{
    public Shooter _shooter;

    public ToggleAmpMode(Shooter shooter) {
        _shooter = shooter;
    }

    @Override
    public void initialize() {
        _shooter.setAmpMode(true);
        _shooter.setToHandoffRPM();

    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        _shooter.setAmpMode(false);
    }
}