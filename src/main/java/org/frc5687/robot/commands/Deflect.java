package org.frc5687.robot.commands;

import org.frc5687.robot.subsystems.Deflector;

public class Deflect extends OutliersCommand{
    private Deflector _deflector;

    public Deflect(Deflector deflector) {
        _deflector = deflector;
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        _deflector.setMotionMagic(_deflector.getTargetAngle());
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

