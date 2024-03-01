package org.frc5687.robot.commands.Shooter;

import org.frc5687.robot.Constants;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.Intake;
import org.frc5687.robot.subsystems.Shooter;

public class ShooterEject extends OutliersCommand {
    private Shooter _shooter;
    private Intake _intake;
    
    public ShooterEject(Shooter shooter, Intake intake) {
        _intake = intake;
        _shooter = shooter;
        addRequirements(_shooter, _intake);
        }

    @Override
    public void execute(){
    
        _shooter.setPercentRPM();
        _intake.setSpeed(Constants.Intake.INTAKE_SPEED);
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

