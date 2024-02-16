package org.frc5687.robot.commands.Shooter;

import org.frc5687.robot.commands.Deflector.SetDeflectorAngle;
import org.frc5687.robot.commands.DriveTrain.DriveToAmp;
import org.frc5687.robot.commands.DriveTrain.DriveToPose;
import org.frc5687.robot.commands.Intake.TimedIntake;
import org.frc5687.robot.subsystems.Deflector;
import org.frc5687.robot.subsystems.DriveTrain;
import org.frc5687.robot.subsystems.Intake;
import org.frc5687.robot.subsystems.Shooter;

import org.frc5687.robot.Constants;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AmpShot extends SequentialCommandGroup{

    public AmpShot(Shooter shooter, Deflector deflector, DriveTrain driveTrain, Intake intake) {        
        addCommands(
            new ParallelCommandGroup(
                new RPMForAmpShot(shooter),
                new SetDeflectorAngle(deflector, Constants.Shooter.AMP_SHOT_DEFLECTOR_ANGLE),
                new DriveToAmp(driveTrain)
            ),
            new DriveToAmp(driveTrain),
            new TimedIntake(intake, 1000)
        );
    }
    
}
