package org.frc5687.robot.commands;

import org.frc5687.robot.Constants;
import org.frc5687.robot.subsystems.DriveTrain;

import edu.wpi.first.math.controller.PIDController;
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
        // TODO Auto-generated method stub
        super.initialize();
    }

    @Override
    public void execute() {
        // TODO Auto-generated method stub
        _driveTrain.setVelocityPose(_goalPose2d);
    }

    @Override
    public boolean isFinished() {
        return (Math.abs(_driveTrain.getEstimatedPose().getX() - _goalPose2d.getX()) < Constants.DriveTrain.POSITION_TOLERANCE) && 
        (Math.abs(_driveTrain.getEstimatedPose().getY() - _goalPose2d.getY()) < Constants.DriveTrain.POSITION_TOLERANCE);

        // TODO: use angle here as well
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

}
