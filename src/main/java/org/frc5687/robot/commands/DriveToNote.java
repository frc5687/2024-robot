package org.frc5687.robot.commands;

import org.frc5687.Messages.VisionPose;
import org.frc5687.Messages.VisionPoseArray;
import org.frc5687.robot.Constants;
import org.frc5687.robot.subsystems.DriveTrain;
import org.frc5687.robot.util.VisionProcessor;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class DriveToNote extends OutliersCommand{
    private final DriveTrain _driveTrain;
    private final PIDController _xController;
    private final PIDController _yController;
    private final VisionProcessor _VisionProcessor;

    public DriveToNote(DriveTrain driveTrain, VisionProcessor visionProcessor) {
        _driveTrain = driveTrain;
        _VisionProcessor = visionProcessor;
        _xController = new PIDController(1.0, 0, 0);
        _yController = new PIDController(1.0, 0, 0);
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
        // TODO Auto-generated method stub
        double vx = 0;
        double vy = 0;
        VisionPoseArray poses = _VisionProcessor.getDetectedObjects();
        VisionPose pose  = null;
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
            error("VisionPose " + 0 + "{ x: " + pose.x() + ", y: " + pose.y() + ", z: " + pose.z() + " }");
            vx = -_xController.calculate(pose.x());
            vy = -_xController.calculate(pose.y());
        }
        metric("Note vx", vx);
        metric("Note vy", vy);
        _driveTrain.setVelocity(new ChassisSpeeds(vx, vy, 0));
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
