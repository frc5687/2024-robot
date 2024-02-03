package org.frc5687.robot.commands.Climber;

import org.frc5687.robot.subsystems.Climber;
import org.frc5687.robot.commands.OutliersCommand;

public class SetClimberPosition extends OutliersCommand {
        private Climber _climber; 
        private double _meters;

    public SetClimberPosition(Climber climber, double meters){
        _climber = climber;
        _meters = meters;
        addRequirements(_climber);
    }
    @Override
    public void initialize() {
        _climber.setPositionMeters(_meters);
    } 
}