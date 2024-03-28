package org.frc5687.robot.commands;

import org.frc5687.robot.subsystems.Intake;
import org.frc5687.robot.subsystems.Shooter;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import org.frc5687.robot.subsystems.DriveTrain;
import org.frc5687.robot.Constants;
import org.frc5687.robot.commands.Shooter.RevShooter;
import org.frc5687.robot.commands.Shooter.AutoShoot;

public class NoteSixCommandChain extends OutliersCommand {

    private Shooter _shooter;
    private Intake _intake;
    private DriveTrain _driveTrain;
    public static boolean noteSixPickup;
    Command _pathfindShootSixCommand;
    
    public NoteSixCommandChain(Shooter shooter, Intake intake, DriveTrain driveTrain) {
        _shooter = shooter;
        _intake = intake;
        _driveTrain = driveTrain;
    }

   
@Override
public void initialize() {

    boolean noteDetected = _intake.isNoteDetected();

        PathPlannerPath sixPath = PathPlannerPath.fromPathFile("pathToShootSix");
        PathConstraints sixConstraints = new PathConstraints(3.0, 4.0,Units.degreesToRadians(540), Units.degreesToRadians(720));
        _pathfindShootSixCommand = AutoBuilder.pathfindThenFollowPath(sixPath,sixConstraints,0.0);

    if (noteDetected == true){
        noteSixPickup = true;
    }
    else if (noteDetected == false){
        noteSixPickup = false;
    }

    if (noteSixPickup == true){
    CommandScheduler.getInstance().schedule(
        new RevShooter(_shooter, Constants.Shooter.IDLE_RPM), 
        _pathfindShootSixCommand,       
        new AutoShoot(_shooter, _intake, _driveTrain, null)
        );
    }

}


    @Override
    public void end(boolean interrupted) { 
    }
}

