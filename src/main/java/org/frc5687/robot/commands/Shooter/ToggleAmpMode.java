package org.frc5687.robot.commands.Shooter;

import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.Shooter;

public class ToggleAmpMode extends OutliersCommand{
    public Shooter _shooter;

    public ToggleAmpMode(Shooter shooter) {
        _shooter = shooter;
    }

    @Override
    public void initialize() {

    }
    

    @Override
    public void execute() {

    }


    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        
        if(_shooter.getAmpMode() == false)
        {_shooter.setToHandoffRPM();
        _shooter.setAmpMode(true);}
        else if(_shooter.getAmpMode() == true)
        {
            _shooter.setAmpMode(false);
            _shooter.setToIdle();
            
        }

    }
}