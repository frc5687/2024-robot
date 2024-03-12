package org.frc5687.robot.commands.DriveTrain;

import java.util.Optional;

import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotState;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.proto.Controller.ProtobufDifferentialDriveWheelVoltages;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import static org.frc5687.robot.Constants.DriveTrain.LOW_KINEMATIC_LIMITS;


public class ReturnToShoot extends Command {
   
    private Command scheduledCommand = null;
    private final RobotState _RobotState = RobotState.getInstance();
    
    
    public ReturnToShoot(){

    }
    @Override
    public void initialize() {
        scheduledCommand = null;
    }
    @Override
    public void execute() {
        Pose2d pose = _RobotState.isRedAlliance() ? Constants.DriveTrain.RED_SHOOT_POSE : Constants.DriveTrain.BLUE_SHOOT_POSE;
        if (scheduledCommand == null) { // we haven't scheduled a command yet

                
                scheduledCommand = AutoBuilder.pathfindToPose(pose, new PathConstraints(
                    LOW_KINEMATIC_LIMITS.maxDriveVelocity,
                    LOW_KINEMATIC_LIMITS.maxDriveAcceleration,
                    LOW_KINEMATIC_LIMITS.maxSteeringVelocity,
                    LOW_KINEMATIC_LIMITS.maxSteeringVelocity * 3
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
