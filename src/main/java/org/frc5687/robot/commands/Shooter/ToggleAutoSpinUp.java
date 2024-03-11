package org.frc5687.robot.commands.Shooter;

import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.Shooter;

public class ToggleAutoSpinUp extends OutliersCommand{
    public Shooter _shooter;

    public ToggleAutoSpinUp(Shooter shooter) {
        _shooter = shooter;
    }

    @Override
    public void initialize() {
        _shooter.setSpinUpAutomatically(!_shooter.getSpinUpAutomatically());
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}