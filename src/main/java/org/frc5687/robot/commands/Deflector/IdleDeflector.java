package org.frc5687.robot.commands.Deflector;

import org.frc5687.robot.subsystems.Deflector;

import edu.wpi.first.wpilibj.DriverStation;

import org.frc5687.robot.Constants;
import org.frc5687.robot.commands.OutliersCommand;

public class IdleDeflector extends OutliersCommand{
    private Deflector _deflector;

    public IdleDeflector(Deflector deflector) {
        _deflector = deflector;
        addRequirements(_deflector);
        DriverStation.reportError("balls balls", false);
    }

    @Override
    public void execute() {
        _deflector.setTargetAngle(Constants.Deflector.IDLE_ANGLE + 0.1);
        DriverStation.reportError("execute", false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}