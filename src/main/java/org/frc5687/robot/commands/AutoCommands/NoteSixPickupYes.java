package org.frc5687.robot.commands.AutoCommands;

import org.frc5687.robot.subsystems.Intake;
import org.frc5687.robot.subsystems.Shooter;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import org.frc5687.robot.subsystems.DriveTrain;
import org.frc5687.robot.Constants;
import org.frc5687.robot.commands.Shooter.RevShooter;
import org.frc5687.robot.commands.Shooter.AutoShoot;

public class NoteSixPickupYes extends SequentialCommandGroup {

    public static boolean noteSixPickup;
    
    PathPlannerPath sixPath = PathPlannerPath.fromPathFile("pathToShootSix");
    PathConstraints sixConstraints = new PathConstraints(3.0, 4.0,Units.degreesToRadians(540), Units.degreesToRadians(720));
    Command _pathfindShootSixCommand = AutoBuilder.pathfindThenFollowPath(sixPath,sixConstraints,0.0);
    
    public NoteSixPickupYes(Shooter shooter, Intake intake, DriveTrain driveTrain) {
        addCommands(
            new NoteIntakedCommand(6,true),
            new ParallelDeadlineGroup(
                _pathfindShootSixCommand,
                new RevShooter(shooter, Constants.Shooter.IDLE_RPM)
            ),    
            new AutoShoot(shooter, intake, driveTrain, null)
        );

    }
}
