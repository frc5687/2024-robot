package org.frc5687.robot.commands.Deflector;

import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.Deflector;

public class ZeroDeflector extends OutliersCommand {
    private Deflector _deflector;

    public ZeroDeflector(Deflector deflector) {
        _deflector = deflector;
    }

    @Override
    public void initialize() {
        _deflector.beginDeflectorZero();
    }

    @Override
    public boolean isFinished() {
        return _deflector.getHall();
    }
    
    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        _deflector.setTargetAngle(1.0);
        _deflector.stopMotor();
    }

}
