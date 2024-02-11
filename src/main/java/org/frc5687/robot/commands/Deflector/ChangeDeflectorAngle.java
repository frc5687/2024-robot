package org.frc5687.robot.commands.Deflector;

import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.Deflector;

public class ChangeDeflectorAngle extends OutliersCommand{
    public Deflector _deflector;
    public double _change;

    public ChangeDeflectorAngle(Deflector deflector, double change) {
        _deflector = deflector;
        _change = change;

        addRequirements(_deflector);
    }

    @Override
    public void initialize() {
        _deflector.setTargetAngle(_deflector.getTargetAngle() + _change);
    }

    @Override
    public boolean isFinished() {
        return _deflector.isAtTargetAngle();
    }

    
}
