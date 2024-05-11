package org.frc5687.robot.commands.DriveTrain;

import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotState;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import static org.frc5687.robot.Constants.DriveTrain.KINEMATIC_LIMITS;


public class ReturnToShootOpposite extends Command {
   
    private Command scheduledCommand = null;
    private final RobotState _robotState = RobotState.getInstance();
    
    
    public ReturnToShootOpposite(){

    }
    @Override
    public void initialize() {
        scheduledCommand = null;
    }
    @Override
    public void execute() {
        Pose2d pose = _robotState.isRedAlliance() ? Constants.DriveTrain.RED_SHOOT_POSE_OPPOSITE : Constants.DriveTrain.BLUE_SHOOT_POSE_OPPOSITE;
        if (scheduledCommand == null) { // we haven't scheduled a command yet

                
                scheduledCommand = AutoBuilder.pathfindToPose(pose, new PathConstraints(
                    KINEMATIC_LIMITS.maxDriveVelocity,
                    KINEMATIC_LIMITS.maxDriveAcceleration,
                    KINEMATIC_LIMITS.maxSteeringVelocity,
                    KINEMATIC_LIMITS.maxSteeringVelocity * 3
                ));
            
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
