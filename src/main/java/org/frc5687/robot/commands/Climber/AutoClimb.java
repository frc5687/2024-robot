package org.frc5687.robot.commands.Climber;

import org.frc5687.robot.subsystems.Climber;
import org.frc5687.robot.commands.OutliersCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoClimb extends OutliersCommand{
    
    private Climber _climber;
    
    public AutoClimb(Climber climber) {
        _climber = climber;
    }

    @Override
    public void initialize() {
        super.initialize();
        SequentialCommandGroup group = new SequentialCommandGroup(); 
        switch(_climber.getStep()) {
            case UNKNOWN:
            case STOW:
            case STOWED:
                group.addCommands(new PrepToClimb(_climber));
                break;
            case READY_TO_CLIMB:
            case RAISE_ARM:
                group.addCommands(new RaiseArm(_climber));
                break;
            case ARM_RAISED:
            case DRIVE_FORWARD:
                //group.addCommands(new DriveForward(_climber));
                //break;
            case LOWER_ARM:
                group.addCommands(new LowerArm(_climber));
            case ARM_LOWERED:  
                _climber.setStep(Climber.ClimberStep.DONE);          
            case DONE:
                break;
        }
        group.schedule();
    }

    @Override
    public boolean isFinished() {
        super.isFinished();
        return true;
    }
}
