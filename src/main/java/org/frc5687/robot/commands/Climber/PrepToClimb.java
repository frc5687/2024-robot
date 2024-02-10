package org.frc5687.robot.commands.Climber;

import org.frc5687.robot.subsystems.Climber;
import org.frc5687.robot.subsystems.Climber.ClimberStep;
import org.frc5687.robot.commands.OutliersCommand;

public class PrepToClimb extends OutliersCommand {
        
    private Climber _climber; 

    public PrepToClimb(Climber climber) {
        _climber = climber;
        addRequirements(_climber);
        info("Instantiated PrepToClimb Check");
    }

    @Override
    public void initialize() {
        super.initialize();
        info("Initialized PrepToClimb");
        _climber.setStep(Climber.ClimberStep.PREP_TO_CLIMB);
    } 

    @Override
    public boolean isFinished() {
        super.isFinished();
        if(_climber.isClimberAtLowerLimit()) {
            info("Finished PrepToClimb.");
            return true;
        } else {
            return false;
        }

    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}