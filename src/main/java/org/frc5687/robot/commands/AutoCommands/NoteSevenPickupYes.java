package org.frc5687.robot.commands.AutoCommands;

import org.frc5687.robot.subsystems.Intake;
import org.frc5687.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import org.frc5687.robot.subsystems.DriveTrain;
import org.frc5687.robot.Constants;
import org.frc5687.robot.commands.DriveTrain.DriveToNote;
import org.frc5687.robot.commands.DriveTrain.DriveToPose;
import org.frc5687.robot.commands.Shooter.AutoPassthrough;
import org.frc5687.robot.commands.Shooter.RevShooter;

public class NoteSevenPickupYes extends SequentialCommandGroup {
    
    public NoteSevenPickupYes(Shooter shooter, Intake intake, DriveTrain driveTrain) {
        addCommands(  
            new NoteIntakedCommand(7,true),
            new RevShooter(shooter, Constants.Shooter.PASSTHROUGH_RPM),        
            new DriveToPose(driveTrain, Constants.DriveTrain.AUTO_POSE_NOTE_SEVEN_PASSTHROUGH),
            new AutoPassthrough(shooter, intake),
            new DriveToPose(driveTrain, Constants.DriveTrain.AUTO_POSE_NOTE_SIX_PICKUP),
            new DriveToNote(driveTrain, intake)
        );
    }
}