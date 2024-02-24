package org.frc5687.robot.commands;

import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotState;
import org.frc5687.robot.subsystems.DriveTrain.DriveTrain;
import org.frc5687.robot.subsystems.DriveTrain.DriveTrain.ControlState;

import edu.wpi.first.math.geometry.Pose2d;

public class DriveToAmp extends OutliersCommand {
    private final DriveTrain _driveTrain;
    private final RobotState _robotState = RobotState.getInstance();
    private Pose2d _goalPose;

    public DriveToAmp(DriveTrain driveTrain) {
        _driveTrain = driveTrain;
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        _driveTrain.setControlState(ControlState.POSITION);
        _goalPose = _driveTrain.isRedAlliance() ? Constants.Shooter.RED_AMP_SHOT_POSE : Constants.Shooter.BLUE_AMP_SHOT_POSE;
        new DriveToPose(_driveTrain, _goalPose).schedule();
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return (Math.abs(_robotState.getEstimatedPose().getX() - _goalPose.getX()) < Constants.DriveTrain.POSITION_TOLERANCE) && 
        (Math.abs(_robotState.getEstimatedPose().getY() - _goalPose.getY()) < Constants.DriveTrain.POSITION_TOLERANCE) &&
        (Math.abs(_robotState.getEstimatedPose().getRotation().minus(_goalPose.getRotation()).getRadians()) < Constants.DriveTrain.HEADING_TOLERANCE);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

}
