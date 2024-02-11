package org.frc5687.robot.commands.Shooter;

import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.Shooter;
import org.frc5687.robot.Constants;

public class RPMForAmpShot extends OutliersCommand{

    private Shooter _shooter;
    
    public RPMForAmpShot(Shooter shooter) {
        _shooter = shooter;
        
        addRequirements(_shooter);
    }

    @Override
    public void initialize() {
        _shooter.setTargetRPM(Constants.Shooter.AMP_SHOT_SPEED);
        _shooter.setToTarget();
    }

    @Override
    public boolean isFinished() {
        return _shooter.isAtTargetRPM();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}
