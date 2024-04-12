package org.frc5687.robot.util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;

import java.util.Optional;

import org.frc5687.robot.Constants;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonProcessor {

    // private static final int NUM_CAMERAS = 3;
    private final PhotonCamera _southEastCamera;
    private final PhotonCamera _southCamera;
    private final PhotonCamera _southWestCamera;
    private final PhotonPoseEstimator _southEastCameraEstimator;
    private final PhotonPoseEstimator _southCameraEstimator;
    private final PhotonPoseEstimator _southWestCameraEstimator;

    private final Transform3d _robotToSouthEastCam;
    private final Transform3d _robotToSouthCam;
    private final Transform3d _robotToSouthWestCam;

    public PhotonProcessor(AprilTagFieldLayout layout) {
        _southEastCamera = new PhotonCamera("South_East_Camera");
        _southCamera = new PhotonCamera("South_Camera");
        _southWestCamera = new PhotonCamera("South_West_Camera");

        _robotToSouthEastCam = new Transform3d(
            -0.235, -0.117, 0.578, 
            new Rotation3d(0, 0, Units.degreesToRadians(162.5))
        );
        _robotToSouthCam = new Transform3d(
            -0.242, 0.0, 0.578, 
            new Rotation3d(0, 0, Units.degreesToRadians(180))
        );
        _robotToSouthWestCam = new Transform3d(
            -0.235, 0.09, 0.578,
            new Rotation3d(0, 0, Units.degreesToRadians(-162.5))
        );

        _southEastCameraEstimator = new PhotonPoseEstimator(
                layout,
                PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                _southEastCamera,
                _robotToSouthEastCam);

        _southCameraEstimator = new PhotonPoseEstimator(
                layout,
                PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                _southCamera,
                _robotToSouthCam);

        _southWestCameraEstimator = new PhotonPoseEstimator(
                layout,
                PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                _southWestCamera,
                _robotToSouthWestCam);

        _southEastCameraEstimator.setMultiTagFallbackStrategy(
                PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
        _southCameraEstimator.setMultiTagFallbackStrategy(
            PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
        _southWestCameraEstimator.setMultiTagFallbackStrategy(
                PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
    }

    public void setLowestAmbiguity() {
        _southEastCameraEstimator.setPrimaryStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
        _southCameraEstimator.setPrimaryStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
        _southWestCameraEstimator.setPrimaryStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
    }

    public Optional<EstimatedRobotPose> getSouthEastCameraEstimatedGlobalPose(
            Pose2d prevEstimatedPose) {
        _southEastCameraEstimator.setReferencePose(prevEstimatedPose);
        PhotonPipelineResult results = _southEastCamera.getLatestResult();
        if (results.hasTargets()) {
            results.targets.removeIf(tag -> tag.getPoseAmbiguity() > Constants.Vision.AMBIGUITY_TOLERANCE);
        }
        return _southEastCameraEstimator.update(results);
    }

    public Optional<EstimatedRobotPose> getSouthCameraEstimatedGlobalPose(
            Pose2d prevEstimatedPose) {
        _southCameraEstimator.setReferencePose(prevEstimatedPose);
        PhotonPipelineResult results = _southCamera.getLatestResult();
        if (results.hasTargets()) {
            results.targets.removeIf(tag -> tag.getPoseAmbiguity() > Constants.Vision.AMBIGUITY_TOLERANCE);
        }
        return _southCameraEstimator.update(results);
    }

    public Optional<EstimatedRobotPose> getSouthWestCameraEstimatedGlobalPose(
            Pose2d prevEstimatedPose) {
        _southWestCameraEstimator.setReferencePose(prevEstimatedPose);
        PhotonPipelineResult results = _southWestCamera.getLatestResult();
        if (results.hasTargets()) {
            results.targets.removeIf(tag -> tag.getPoseAmbiguity() > Constants.Vision.AMBIGUITY_TOLERANCE);
        }
        return _southWestCameraEstimator.update(results);
    }

    public Pair<EstimatedRobotPose, String> getSouthEastCameraEstimatedGlobalPoseWithName(
            Pose2d prevEstimatedPose) {
        _southEastCameraEstimator.setReferencePose(prevEstimatedPose);
        PhotonPipelineResult results = _southEastCamera.getLatestResult();
        if (results.hasTargets()) {
            results.targets.removeIf(tag -> tag.getPoseAmbiguity() > Constants.Vision.AMBIGUITY_TOLERANCE);
        }
        Optional<EstimatedRobotPose> pose = _southEastCameraEstimator.update(results);
        return new Pair<>(pose.orElse(null), "SouthEast");
    }

    public Pair<EstimatedRobotPose, String> getSouthCameraEstimatedGlobalPoseWithName(
            Pose2d prevEstimatedPose) {
        _southCameraEstimator.setReferencePose(prevEstimatedPose);
        PhotonPipelineResult results = _southCamera.getLatestResult();
        if (results.hasTargets()) {
            results.targets.removeIf(tag -> tag.getPoseAmbiguity() > Constants.Vision.AMBIGUITY_TOLERANCE);
        }
        Optional<EstimatedRobotPose> pose = _southCameraEstimator.update(results);
        return new Pair<>(pose.orElse(null), "NorthEast");
    }

    public Pair<EstimatedRobotPose, String> getSouthWestCameraEstimatedGlobalPoseWithName(
            Pose2d prevEstimatedPose) {
        _southWestCameraEstimator.setReferencePose(prevEstimatedPose);
        PhotonPipelineResult results = _southWestCamera.getLatestResult();
        if (results.hasTargets()) {
            results.targets.removeIf(tag -> tag.getPoseAmbiguity() > Constants.Vision.AMBIGUITY_TOLERANCE);
        }
        Optional<EstimatedRobotPose> pose = _southWestCameraEstimator.update(results);
        return new Pair<>(pose.orElse(null), "SouthWest");
    }

    public Optional<Rotation2d> calculateAngleToTag(int tagId) {
        Optional<Rotation2d> southYaw = Optional.empty();
        for (PhotonTrackedTarget target : _southCamera.getLatestResult().targets) {
            if (target.getFiducialId() == tagId) {
                southYaw = Optional.of(Rotation2d.fromDegrees(target.getYaw()));
                break;
            }
        }
        return southYaw;
    }

    public Optional<Double> calculateDistanceToTag(int tagId) {
        Optional<Double> southDist = Optional.empty();
        for (PhotonTrackedTarget target : _southCamera.getLatestResult().targets) {
            if (target.getFiducialId() == tagId) {
                var camToTarget = target.getBestCameraToTarget();
                var robotToCamera = _robotToSouthCam;
                var robotToTarget = robotToCamera.plus(camToTarget);

                double distance = Math.sqrt(Math.pow(robotToTarget.getX(), 2) + Math.pow(robotToTarget.getY(), 2));
                southDist = Optional.of(distance);
                break;
            }
        }
        return southDist;
    }

    public enum Pipeline {
        FAR(0),
        CLOSE(1);

        private final int _value;

        Pipeline(int value) {
            _value = value;
        }

        public int getValue() {
            return _value;
        }
    }
}
