package org.frc5687.robot.commands;

import org.frc5687.robot.subsystems.Intake;
import org.frc5687.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.CommandScheduler;

import org.frc5687.robot.subsystems.DriveTrain;
import org.frc5687.robot.Constants;
import org.frc5687.robot.commands.DriveTrain.DriveToNote;
import org.frc5687.robot.commands.DriveTrain.DriveToPose;
import org.frc5687.robot.commands.Shooter.AutoShoot;
import org.frc5687.robot.commands.Shooter.RevShooter;

public class NearSourceBloopCommandChain extends OutliersCommand {

    private Shooter _shooter;
    private Intake _intake;
    private DriveTrain _driveTrain;
    
    public NearSourceBloopCommandChain(Shooter shooter, Intake intake, DriveTrain driveTrain) {
        _shooter = shooter;
        _intake = intake;
        _driveTrain = driveTrain;
       
    }

    @Override
    public void initialize() {

        if (NoteEightCommandChain.noteEightPickup == true){
            CommandScheduler.getInstance().schedule(        
                new DriveToPose(_driveTrain, Constants.DriveTrain.AUTO_POSE_NOTE_FUIC_NEAR_SOURCE),
                new DriveToNote(_driveTrain, _intake),
                new RevShooter(_shooter),
                new DriveToPose(_driveTrain, Constants.DriveTrain.AUTO_POSE_SHOOT_SOURCE),
                new AutoShoot(_shooter, _intake, _driveTrain, null)
                );
            }
        else if (NoteEightCommandChain.noteEightPickup == false){

        }

    }
    @Override
    public void end(boolean interrupted) { 
    }
}