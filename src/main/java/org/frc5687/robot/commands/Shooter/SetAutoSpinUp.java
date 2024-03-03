package org.frc5687.robot.commands.Shooter;

import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.Shooter;

public class SetAutoSpinUp extends OutliersCommand{
    public Shooter _shooter;
    public boolean _value;

    public SetAutoSpinUp(Shooter shooter, boolean value) {
        _shooter = shooter;
        _value = value;
    }

    @Override
    public void initialize() {
        _shooter.setSpinUpAutomatically(_value);
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