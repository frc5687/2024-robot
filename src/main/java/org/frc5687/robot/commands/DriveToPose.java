package org.frc5687.robot.commands;

import org.frc5687.robot.subsystems.DriveTrain.DriveTrain;

import edu.wpi.first.math.geometry.Pose2d;

public class DriveToPose extends OutliersCommand{
    private final DriveTrain _driveTrain;
    private final Pose2d _goalPose2d;

    public DriveToPose(DriveTrain driveTrain, Pose2d goalPose2d){
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
    }

    @Override
    public boolean isFinished() {
        return _driveTrain.isAutoAlignComplete();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

}
