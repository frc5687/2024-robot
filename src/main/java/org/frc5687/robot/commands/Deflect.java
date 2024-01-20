package org.frc5687.robot.commands;

import org.frc5687.robot.subsystems.Deflector;

public class Deflect extends OutliersCommand{
    private Deflector _deflector;

    public Deflect(Deflector deflector) {
        _deflector = deflector;
        addRequirements(_deflector);
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        System.out.println("Setting motion magic to "+_deflector.getTargetAngle()+" degrees.");
        _deflector.setMotionMagic(_deflector.getTargetAngle());
    }

    @Override
    public boolean isFinished() {
        return false; 
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}

