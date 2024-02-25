package org.frc5687.robot.commands;

import org.frc5687.Messages.VisionPose;
import org.frc5687.Messages.VisionPoseArray;
import org.frc5687.robot.Constants;
import org.frc5687.robot.subsystems.DriveTrain.DriveTrain;
import org.frc5687.robot.util.VisionProcessor;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public class DriveToNote extends OutliersCommand {
    private final DriveTrain _driveTrain;
    private final ProfiledPIDController _xController;
    private final ProfiledPIDController _yController;
    private final ProfiledPIDController _yawController;
    private final VisionProcessor _visionProcessor;

    public DriveToNote(DriveTrain driveTrain, VisionProcessor visionProcessor) {
        _driveTrain = driveTrain;
        _visionProcessor = visionProcessor;
        _xController = new ProfiledPIDController(3.0, 0.0, 0.0,
                new Constraints(Constants.DriveTrain.SLOW_KINEMATIC_LIMITS.maxDriveVelocity,
                        Constants.DriveTrain.SLOW_KINEMATIC_LIMITS.maxDriveAcceleration));
        _yController = new ProfiledPIDController(3.0, 0.0, 0.0,
                new Constraints(Constants.DriveTrain.SLOW_KINEMATIC_LIMITS.maxDriveVelocity,
                        Constants.DriveTrain.SLOW_KINEMATIC_LIMITS.maxDriveAcceleration));
        _yawController = new ProfiledPIDController(3.0, 0.0, 0.0,
                new Constraints(Constants.DriveTrain.MAX_ANG_VEL, Constants.DriveTrain.MAX_ANG_VEL * 4.0));
        addRequirements(_driveTrain);
    }

    @Override
    public void initialize() {
        super.initialize();
        _xController.setGoal(0.0);
        _yController.setGoal(0.0);
        _yawController.setGoal(0.0);
    }

    @Override
    public void execute() {
        double vx = 0;
        double vy = 0;
        double rot = 0;
        VisionPoseArray poses = _visionProcessor.getDetectedObjects();
        VisionPose pose = null;

        if (!_driveTrain.isLowGear()) {
            _driveTrain.shiftDownModules();
        }

        metric("Jetson Note Detected", poses.posesLength() > 0);
        for (int i = 0; i < poses.posesLength(); i++) {
            if (pose == null) {
                pose = poses.poses(i);
            } else {
                if (poses.poses(i).x() < pose.x() && poses.poses(i).y() < pose.y()) {
                    pose = poses.poses(i);
                } else {
                    if (poses.poses(i).x() < pose.x()) {
                        pose = poses.poses(i);
                    }
                }
            }
        }
        if (pose != null) {
            if (Double.isNaN(pose.x()) || Double.isNaN(pose.y())) {
                vx = 0;
                vy = 0;
                rot = 0;
            } else {
                /* Drive to note portion */
                double x = pose.x();
                double y = pose.y() - 0.0285; // make constant for ZED y offset

                vx = -_xController.calculate(x);
                vy = -_xController.calculate(y);

                // FIXME use IMU data

                double angleToNoteRadians = Math.atan2(y, x);

                metric("Note x", x);
                metric("Note y", y);
                metric("Angle to note", angleToNoteRadians);
                rot = -_yawController.calculate(angleToNoteRadians);
            }
        } else {
            error(" _visionProcessor.getDetectedObjects() returned null ");
        }
        metric("Note vx", vx);
        metric("Note vy", vy);
        _driveTrain.setVelocity(new ChassisSpeeds(vx, vy, rot));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

}
