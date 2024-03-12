package org.frc5687.robot.commands.Shooter;

import org.frc5687.robot.commands.Intake.TimedIntake;
import org.frc5687.robot.subsystems.DriveTrain;
import org.frc5687.robot.subsystems.Intake;
import org.frc5687.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AmpShot extends SequentialCommandGroup{

    public AmpShot(Shooter shooter, DriveTrain driveTrain, Intake intake) {        
        addCommands(
            new TimedIntake(intake, 1000)
        );
    }
    
}
