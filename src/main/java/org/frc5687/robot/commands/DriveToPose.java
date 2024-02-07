package org.frc5687.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import org.frc5687.robot.subsystems.DriveTrain;

import static org.frc5687.robot.Constants.DriveTrain.DRIVE_POSE_KINEMATIC_LIMITS;
import static org.frc5687.robot.Constants.DriveTrain.KINEMATIC_LIMITS;

public class DriveToPose extends OutliersCommand {

    private final DriveTrain _driveTrain;
    private Pose2d _destPose;

    public DriveToPose(DriveTrain driveTrain, Pose2d pose) {
        _driveTrain = driveTrain;
        _destPose = pose;
        addRequirements(_driveTrain);
    }

    @Override
    public void initialize() {
        super.initialize();
        _driveTrain.setKinematicLimits(DRIVE_POSE_KINEMATIC_LIMITS);
        //        _destPose = new Pose2d(_destPose.getX(), _destPose.getY(), _driveTrain.getHeading());
    }

    @Override
    public void execute() {
        _driveTrain.setVelocityPose(_destPose);
    }

    @Override
    public boolean isFinished() {
        double xDiff = _destPose.getX() - _driveTrain.getEstimatedPose().getX();
        double yDiff = _destPose.getY() - _driveTrain.getEstimatedPose().getY();
        return (Math.abs(xDiff) < 0.03 && Math.abs(yDiff) < 0.03);
    }

    @Override
    public void end(boolean interrupted) {
        _driveTrain.setKinematicLimits(KINEMATIC_LIMITS);
        super.end(interrupted);
        error("Drive to Pose Ended!");
    }
}
