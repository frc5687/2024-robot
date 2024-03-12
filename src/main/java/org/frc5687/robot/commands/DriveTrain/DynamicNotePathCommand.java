package org.frc5687.robot.commands.DriveTrain;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;

import static org.frc5687.robot.Constants.DriveTrain.LOW_KINEMATIC_LIMITS;
import java.util.Optional;

import org.frc5687.robot.RobotState;

public class DynamicNotePathCommand extends Command {
    private final RobotState _robotState = RobotState.getInstance();
    private Command scheduledCommand = null;

    public DynamicNotePathCommand() {
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (scheduledCommand == null) { // we haven't scheduled a command yet
            Optional<Pose2d> closestNotePose = _robotState.getClosestNoteRelativeField();

            if (closestNotePose.isPresent()) {
                // If there's a closest note, schedule pathfinding to it
                scheduledCommand = AutoBuilder.pathfindToPose(closestNotePose.get(), new PathConstraints(
                    LOW_KINEMATIC_LIMITS.maxDriveVelocity,
                    LOW_KINEMATIC_LIMITS.maxDriveAcceleration,
                    LOW_KINEMATIC_LIMITS.maxSteeringVelocity,
                    LOW_KINEMATIC_LIMITS.maxSteeringVelocity * 3
                ));
            } else {
                scheduledCommand = new WaitCommand(0);// just end
            }

            CommandScheduler.getInstance().schedule(scheduledCommand);
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (scheduledCommand != null && scheduledCommand.isScheduled()) {
            scheduledCommand.cancel();
        }
    }

    @Override
    public boolean isFinished() {
        return scheduledCommand != null && !scheduledCommand.isScheduled();
    }
}


