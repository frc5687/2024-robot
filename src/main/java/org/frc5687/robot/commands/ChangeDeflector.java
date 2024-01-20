package org.frc5687.robot.commands;

import org.frc5687.robot.Constants;
import org.frc5687.robot.subsystems.Deflector;
import org.frc5687.robot.subsystems.Intake;

public class ChangeDeflector extends OutliersCommand{
    private Deflector _deflector;

    public ChangeDeflector(Deflector deflector, double change_in_angle) {
        _deflector = deflector;
        _deflector.setTargetAngle(_deflector.getTargetAngle() + change_in_angle);
    }

    @Override
    public void initialize() {
        super.initialize();
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

