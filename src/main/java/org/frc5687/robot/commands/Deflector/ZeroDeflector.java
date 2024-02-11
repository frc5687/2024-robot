package org.frc5687.robot.commands.Deflector;

import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.Deflector;

import edu.wpi.first.wpilibj.DriverStation;

public class ZeroDeflector extends OutliersCommand {
    private Deflector _deflector;

    public ZeroDeflector(Deflector deflector) {
        _deflector = deflector;
        addRequirements(_deflector);
    }

    @Override
    public void initialize() {
        DriverStation.reportError("haiii :3", false);
        _deflector.beginDeflectorZero();
    }

    @Override
    public void execute() {
    }
    @Override
    public boolean isFinished() {
        
        return _deflector.getHall();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        _deflector.stopMotor();
        _deflector.setTargetAngle(1.0);
        DriverStation.reportError("jeepers creepers", false);
    }

}
