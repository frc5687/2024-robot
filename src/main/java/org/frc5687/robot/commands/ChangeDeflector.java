package org.frc5687.robot.commands;

import org.frc5687.robot.subsystems.Deflector;

public class ChangeDeflector extends OutliersCommand{
    private Deflector _deflector;
    private final double _changeInAngle;

    public ChangeDeflector(Deflector deflector, double changeInAngle) {
        _deflector = deflector;
        _changeInAngle = changeInAngle;
    }

    @Override
    public void initialize() {
        super.initialize();
        System.out.println("Setting angle to "+(_deflector.getTargetAngle() + _changeInAngle)+".");
        _deflector.setTargetAngle(_deflector.getTargetAngle() + _changeInAngle);
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

