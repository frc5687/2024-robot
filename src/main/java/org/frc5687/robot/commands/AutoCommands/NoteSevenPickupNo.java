package org.frc5687.robot.commands.AutoCommands;

import org.frc5687.robot.subsystems.Intake;
import org.frc5687.robot.subsystems.Shooter;
import org.frc5687.robot.util.PhotonObjectProcessor;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import org.frc5687.robot.subsystems.DriveTrain;
import org.frc5687.robot.Constants;
import org.frc5687.robot.commands.DriveTrain.DriveToNote;
import org.frc5687.robot.commands.DriveTrain.DriveToPose;
import org.frc5687.robot.commands.Shooter.AutoPassthrough;
import org.frc5687.robot.commands.Shooter.RevShooter;

public class NoteSevenPickupNo extends SequentialCommandGroup {

    
    public NoteSevenPickupNo(Shooter shooter, Intake intake, DriveTrain driveTrain, PhotonPipelineResult intakeCamera, PhotonObjectProcessor intakeCameraProcessor) {
        addCommands( 
            new NoteIntakedCommand(7,false),
            new DriveToPose(driveTrain, Constants.DriveTrain.AUTO_POSE_NOTE_SIX_PICKUP),
            new DriveToNote(driveTrain, intake, intakeCamera, intakeCameraProcessor)
        );
    }

}