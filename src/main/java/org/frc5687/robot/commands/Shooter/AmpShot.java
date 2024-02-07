package org.frc5687.robot.commands.Shooter;

import org.frc5687.robot.commands.DriveToPose;
import org.frc5687.robot.commands.Deflector.ChangeDeflectorAngle;
import org.frc5687.robot.subsystems.Deflector;
import org.frc5687.robot.subsystems.DriveTrain;
import org.frc5687.robot.subsystems.Intake;
import org.frc5687.robot.subsystems.Shooter;
import org.frc5687.robot.Constants;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AmpShot extends SequentialCommandGroup{
    public AmpShot(Shooter shooter, Deflector deflector, DriveTrain driveTrain, Intake intake) {
        addCommands(
            new ParallelCommandGroup(
                new RPMForAmpShot(shooter),
                new ChangeDeflectorAngle(deflector, Constants.Shooter.AMP_SHOT_DEFLECTOR_ANGLE),
                new DriveToPose(driveTrain, Constants.Shooter.RED_AMP_SHOT_POSE)
            ),
            new Shoot(shooter, intake)
        );
    }
    
}
