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
        super.initialize();
    }

    public void execute() {
        super.execute();
        _climber.setSpeed(_oi.getClimbY());
    }

}
