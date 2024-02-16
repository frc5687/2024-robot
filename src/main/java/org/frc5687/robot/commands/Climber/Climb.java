package org.frc5687.robot.commands.Climber;

import org.frc5687.robot.OI;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.Climber;

public class Climb extends OutliersCommand {
    
    private Climber _climber;
    private OI _oi;

    public Climb(Climber climber, OI oi) {
        _climber = climber;
        _oi = oi;
        addRequirements(_climber);
    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        super.initialize();
    }
    
    @Override
    public void execute() {
        super.execute();
        double speed = _oi.getClimbY();
        _climber.setSpeed(speed);
        metric("ClimberSpeed", speed);
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return super.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        _climber.setSpeed(0);
        super.end(interrupted);
    }

}

