package org.frc5687.robot.commands.Intake;

import org.frc5687.robot.Constants;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.Intake;

public class IndexNote extends OutliersCommand {
    
    private final Intake _intake;
    
    public IndexNote(Intake intake) {
        _intake = intake;
        addRequirements(_intake);
    }
    @Override
    public void initialize() {
        super.initialize();
        
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
       return _intake.isTopDetected();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        _intake.setSpeed(0);
    }
}
