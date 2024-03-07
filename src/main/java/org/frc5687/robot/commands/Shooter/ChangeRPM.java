package org.frc5687.robot.commands.Shooter;

import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.Shooter;

public class ChangeRPM extends OutliersCommand{
    public Shooter _shooter;
    public double _value;

    public ChangeRPM(Shooter shooter, double value) {
        _shooter = shooter;
        _value = value;
    }

    @Override
    public void initialize() {
        _shooter.setManualShootRPM(_shooter.getManualShootRPM() + _value);
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