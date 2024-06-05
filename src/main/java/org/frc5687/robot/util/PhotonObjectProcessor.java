package org.frc5687.robot.util;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import org.frc5687.robot.RobotState;
public class PhotonObjectProcessor {
    
    private final PhotonCamera _intakeCamera;

    private final Transform3d _robotToIntakeCam;

    public PhotonObjectProcessor(){
        _intakeCamera = new PhotonCamera("Intake_Camera");

        _robotToIntakeCam = new Transform3d();
    }

    public Optional<Pose2d> getObjectPose(){
        Optional<Pose2d> pose = Optional.empty();
        Optional<Double> r = calculateDistanceToTarget();
        double objectX = r.get() * Math.cos(getIntakeTargetYaw().get());
        double objectY = r.get() * Math.sin(getIntakeTargetYaw().get());
        Translation2d objectTranslation2d = new Translation2d(objectX, objectY);
        Rotation2d rotation = new Rotation2d(getIntakeTargetYaw().get());
        pose = Optional.of(new Pose2d(objectTranslation2d, rotation));
        return pose;
    }

    public Optional<Double> getIntakeTargetYaw(){
        Optional<Double> targetYaw = Optional.empty();
        for(PhotonTrackedTarget target : _intakeCamera.getLatestResult().targets){
            PhotonPipelineResult results = _intakeCamera.getLatestResult();
            if(results.hasTargets()){
                target = results.getBestTarget();
                double yaw = Units.degreesToRadians(target.getYaw());
                targetYaw = Optional.of(yaw);
                break;
            }
        }
        return targetYaw;
    }

    public Optional<Double> getIntakeTargetPitch(){
        Optional<Double> targetPitch = Optional.empty();
        for(PhotonTrackedTarget target : _intakeCamera.getLatestResult().targets){
            PhotonPipelineResult results = _intakeCamera.getLatestResult();
            if(results.hasTargets()){
                target = results.getBestTarget();
                double pitch = Units.degreesToRadians(target.getPitch());
                targetPitch = Optional.of(pitch);
                break;
            }
        }
        return targetPitch;
    }

    public Optional<Double> calculateDistanceToTarget(){
        Optional<Double> dist = Optional.empty();
        
        double mountAngle = 0.0;
        double camHeight = _robotToIntakeCam.getY();
        Optional<Double> angleToTarget = getIntakeTargetPitch();
        dist = Optional.of((camHeight - 0) / Math.tan(mountAngle + angleToTarget.get())); 
        // camera height from ground - object height from ground divided by tan(cam mount angle + angle to object)
        return dist;
    }
}
    