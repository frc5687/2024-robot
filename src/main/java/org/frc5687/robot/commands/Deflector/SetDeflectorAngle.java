package org.frc5687.robot.commands.Deflector;

import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.Deflector;

public class SetDeflectorAngle extends OutliersCommand{
    private Deflector _deflector;
    private double _angle;

    public SetDeflectorAngle(Deflector deflector, double angle) {
        _deflector = deflector;
        _angle = angle;
        addRequirements(_deflector);
    }

    @Override
    public void initialize() {
        _deflector.setTargetAngle(_angle);
    }  

    @Override
    public boolean isFinished() {
        return _deflector.isAtTargetAngle(); 
    }
}

