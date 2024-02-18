package org.frc5687.robot.commands;

import org.frc5687.Messages.VisionPose;
import org.frc5687.Messages.VisionPoseArray;
import org.frc5687.robot.Constants;
import org.frc5687.robot.subsystems.DriveTrain.DriveTrain;
import org.frc5687.robot.util.VisionProcessor;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public class DriveToNote extends OutliersCommand{
    private final DriveTrain _driveTrain;
    private final ProfiledPIDController _xController;
    private final ProfiledPIDController _yController;
    private final ProfiledPIDController _yawController;
    private final VisionProcessor _visionProcessor;

    public DriveToNote(DriveTrain driveTrain, VisionProcessor visionProcessor) {
        _driveTrain = driveTrain;
        _visionProcessor = visionProcessor;
        _xController = new ProfiledPIDController(2.0, 0.0, 0.0, new Constraints(Constants.DriveTrain.SLOW_KINEMATIC_LIMITS.maxDriveVelocity, Constants.DriveTrain.SLOW_KINEMATIC_LIMITS.maxDriveAcceleration));
        _yController = new ProfiledPIDController(2.0, 0.0, 0.0, new Constraints(Constants.DriveTrain.SLOW_KINEMATIC_LIMITS.maxDriveVelocity, Constants.DriveTrain.SLOW_KINEMATIC_LIMITS.maxDriveAcceleration));
        _yawController = new ProfiledPIDController(3.0, 0.0, 0.0, new Constraints(Constants.DriveTrain.MAX_ANG_VEL, Constants.DriveTrain.MAX_ANG_VEL*4.0));
        addRequirements(_driveTrain);
    }

    @Override
    public void initialize() {
        super.initialize();
        _xController.setGoal(0.0);
        _yController.setGoal(0.0);
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
        if (poses != null) {
            metric("Jetson Note Detected", poses.posesLength() > 0);
            for (int i = 0; i < poses.posesLength(); i++) {
                if (pose == null) {
                    pose = poses.poses(i);
                } else {
                    if (poses.poses(i).x() < pose.x()) {
                        pose = poses.poses(i);
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
                    vx = -_xController.calculate(pose.x());
                    vy = -_yController.calculate(pose.y() + 0.06);

                    // error("VisionPose " + 0 + "{ x: " + pose.x() + ", y: " + pose.y() + ", z: " +
                    // pose.z() + " }");
                    /* angle to note */
                    double angle = Math.atan2(pose.y() + 0.06, pose.x());
                    metric("Note x", pose.x());
                    metric("Note y", pose.y() + 0.06);
                    metric("Angle to note", angle);
                    rot = -_yawController.calculate(angle);
                }
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
