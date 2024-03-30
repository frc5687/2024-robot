package org.frc5687.robot.commands.AutoCommands;

import org.frc5687.robot.subsystems.Intake;
import org.frc5687.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import org.frc5687.robot.subsystems.DriveTrain;
import org.frc5687.robot.Constants;
import org.frc5687.robot.commands.DriveTrain.DriveToNote;
import org.frc5687.robot.commands.DriveTrain.DriveToPose;
import org.frc5687.robot.commands.Shooter.AutoShoot;
import org.frc5687.robot.commands.Shooter.RevShooter;

public class NearSourceBloopPickup extends SequentialCommandGroup {


    public NearSourceBloopPickup(Shooter shooter, Intake intake, DriveTrain driveTrain) {
        addCommands(
                new DriveToPose(driveTrain, Constants.DriveTrain.AUTO_POSE_NOTE_FUIC_NEAR_SOURCE),
                new DriveToNote(driveTrain, intake),
                new ParallelDeadlineGroup(new DriveToPose(driveTrain, Constants.DriveTrain.AUTO_POSE_SHOOT_SOURCE),
                new RevShooter(shooter)),
                new DriveToPose(driveTrain, Constants.DriveTrain.AUTO_POSE_SHOOT_SOURCE),
                new AutoShoot(shooter, intake, driveTrain, null)
        );
    }

}