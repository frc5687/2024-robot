package org.frc5687.robot.commands;

import org.frc5687.Messages.VisionPose;
import org.frc5687.Messages.VisionPoseArray;
import org.frc5687.robot.subsystems.DriveTrain.DriveTrain;
import org.frc5687.robot.util.VisionProcessor;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class DriveToNote extends OutliersCommand{
    private final DriveTrain _driveTrain;
    private final PIDController _xController;
    private final PIDController _yController;
    private final PIDController _yawController;
    private final VisionProcessor _VisionProcessor;

    public DriveToNote(DriveTrain driveTrain, VisionProcessor visionProcessor) {
        _driveTrain = driveTrain;
        _VisionProcessor = visionProcessor;
        _xController = new PIDController(1.0, 0, 0);
        _yController = new PIDController(1.0, 0, 0);
        _yawController = new PIDController(1.0, 0, 0);
        addRequirements(_driveTrain);
    }

    @Override
    public void initialize() {
        super.initialize();
        _xController.setSetpoint(0);
        _yController.setSetpoint(0);
    }

    @Override
    public void execute() {
        double vx = 0;
        double vy = 0;
        double rot = 0;
        VisionPoseArray poses = _VisionProcessor.getDetectedObjects();
        VisionPose pose = null;
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
                vy = -_xController.calculate(pose.y());

                // error("VisionPose " + 0 + "{ x: " + pose.x() + ", y: " + pose.y() + ", z: " +
                // pose.z() + " }");
                /* angle to note */
                double angle = Math.atan2(pose.y(), pose.x());
                metric("Note x", pose.x());
                metric("Note y", pose.y());
                metric("Angle to note", angle);
                rot = -_yawController.calculate(angle);
            }
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
