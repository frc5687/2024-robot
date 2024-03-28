package org.frc5687.robot.commands.TrapMech;

import org.frc5687.robot.commands.Dunker.SetupForTrap;
import org.frc5687.robot.subsystems.Climber;
import org.frc5687.robot.subsystems.DriveTrain;
import org.frc5687.robot.subsystems.Dunker;
import org.frc5687.robot.subsystems.Intake;
import org.frc5687.robot.subsystems.Shooter;
import org.frc5687.robot.subsystems.TrapMech;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class SetupThenTrap extends SequentialCommandGroup {
    
    public SetupThenTrap(Dunker dunker, Shooter shooter, Intake intake, TrapMech trap, Climber climber, DriveTrain driveTrain) {
        addCommands(
            new SetupForTrap(dunker, shooter, intake),
            new TrapSequence(dunker, trap, climber, driveTrain)
        );
    }
}
