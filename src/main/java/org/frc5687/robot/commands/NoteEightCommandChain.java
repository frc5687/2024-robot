package org.frc5687.robot.commands;

import org.frc5687.robot.subsystems.Intake;
import org.frc5687.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.CommandScheduler;

import org.frc5687.robot.subsystems.DriveTrain;
import org.frc5687.robot.Constants;
import org.frc5687.robot.commands.DriveTrain.DriveToNote;
import org.frc5687.robot.commands.DriveTrain.DriveToPose;
import org.frc5687.robot.commands.Shooter.AutoPassthrough;
import org.frc5687.robot.commands.Shooter.RevShooter;

public class NoteEightCommandChain extends OutliersCommand {

    private Shooter _shooter;
    private Intake _intake;
    private DriveTrain _driveTrain;
    public static boolean noteEightPickup;
    
    public NoteEightCommandChain(Shooter shooter, Intake intake, DriveTrain driveTrain) {
        _shooter = shooter;
        _intake = intake;
        _driveTrain = driveTrain;
       
    }

    @Override
    public void initialize() {

        boolean noteDetected = _intake.isNoteDetected();

        if (noteDetected == true){
            noteEightPickup = true;
        }
        else if (noteDetected == false){
            noteEightPickup = false;
        }

        if (noteEightPickup == true){
        CommandScheduler.getInstance().schedule(        
            new RevShooter(_shooter, Constants.Shooter.PASSTHROUGH_RPM),
            new DriveToPose(_driveTrain, Constants.DriveTrain.AUTO_POSE_NOTE_EIGHT_PASSTHROUGH),
            new AutoPassthrough(_shooter, _intake),
            new DriveToPose(_driveTrain, Constants.DriveTrain.AUTO_POSE_NOTE_SEVEN_PICKUP),
            new DriveToNote(_driveTrain, _intake)
            );
        }
        else if (noteEightPickup == false){
        CommandScheduler.getInstance().schedule(        
            new DriveToPose(_driveTrain, Constants.DriveTrain.AUTO_POSE_NOTE_SEVEN_PICKUP),
            new DriveToNote(_driveTrain, _intake)
            );
        }

    }
    @Override
    public void end(boolean interrupted) { 
    }
}