package org.frc5687.robot.commands.DriveTrain;

import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotState;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.DriveTrain;

import edu.wpi.first.math.geometry.Pose2d;

public class DriveToPose extends OutliersCommand {
    private final DriveTrain _driveTrain;
    private final Pose2d _goalPose2d;
    private final RobotState _robotState = RobotState.getInstance();

    public DriveToPose(DriveTrain driveTrain, Pose2d goalPose2d) {
        _driveTrain = driveTrain;
        _goalPose2d = goalPose2d;
    }

    @Override
    public void initialize() {
        _driveTrain.setHoverGoal(_goalPose2d);
        super.initialize();
    }

    @Override
    public void execute() {
        _driveTrain.setVelocityPose(_goalPose2d);
    }

    @Override
    public boolean isFinished() {
        return (Math.abs(_robotState.getEstimatedPose().getX() - _goalPose2d.getX()) < Constants.DriveTrain.POSITION_TOLERANCE) && 
        (Math.abs(_robotState.getEstimatedPose().getY() - _goalPose2d.getY()) < Constants.DriveTrain.POSITION_TOLERANCE) &&
        (Math.abs(_robotState.getEstimatedPose().getRotation().minus(_goalPose2d.getRotation()).getRadians()) < Constants.DriveTrain.ANGLED_HEADING_TOLERANCE); // use angled tolerance because it's smaller idk
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

}
