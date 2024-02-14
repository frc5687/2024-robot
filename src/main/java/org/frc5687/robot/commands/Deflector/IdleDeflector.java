package org.frc5687.robot.commands.Deflector;

import org.frc5687.robot.subsystems.Deflector;
import org.frc5687.robot.Constants;
import org.frc5687.robot.commands.OutliersCommand;

public class IdleDeflector extends OutliersCommand{
    private Deflector _deflector;

    public IdleDeflector(Deflector deflector) {
        _deflector = deflector;
        addRequirements(_deflector);
    }

    @Override
    public void execute() {
        _deflector.setTargetAngle(Constants.Deflector.IDLE_ANGLE);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}