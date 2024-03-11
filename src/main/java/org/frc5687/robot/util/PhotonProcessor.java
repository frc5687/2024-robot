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

    // private static final int NUM_CAMERAS = 4;
    private final PhotonCamera _northEastCamera;
    private final PhotonCamera _southEastCamera;
    private final PhotonCamera _northWestCamera;
    private final PhotonCamera _southWestCamera;
    private final PhotonPoseEstimator _southEastCameraEstimator;
    private final PhotonPoseEstimator _northEastCameraEstimator;
    private final PhotonPoseEstimator _northWestCameraEstimator;
    private final PhotonPoseEstimator _southWestCameraEstimator;

    private final Transform3d _robotToSouthEastCam;
    private final Transform3d _robotToNorthEastCam;
    private final Transform3d _robotToNorthWestCam;
    private final Transform3d _robotToSouthWestCam;

    public PhotonProcessor(AprilTagFieldLayout layout) {
        _southEastCamera = new PhotonCamera("South_East_Camera");
        _northEastCamera = new PhotonCamera("North_East_Camera");
        _northWestCamera = new PhotonCamera("North_West_Camera");
        _southWestCamera = new PhotonCamera("South_West_Camera");

        // FIXME: look at the order the rotation transformations are applied -xavier
        // bradford
        _robotToSouthEastCam = new Transform3d(
                // new Translation3d(0.0, -0.155635, 0.5937), // for centered camera
                new Translation3d(-0.107009, -0.104835, 0.57991),
                // new Rotation3d(0.0, 0.0, Units.degreesToRadians(180)) // for centered camera
                new Rotation3d(0.0, 0.0, Units.degreesToRadians(162.5)));

        _robotToNorthEastCam = new Transform3d(
                new Translation3d(Units.inchesToMeters(3.90), Units.inchesToMeters(-7.05), Units.inchesToMeters(11.00)),
                new Rotation3d(0.0, Units.degreesToRadians(16.5), Units.degreesToRadians(-25.5)));

        _robotToNorthWestCam = new Transform3d(
                new Translation3d(Units.inchesToMeters(3.90), Units.inchesToMeters(7.05), Units.inchesToMeters(11.00)),
                new Rotation3d(0.0, Units.degreesToRadians(16.5), Units.degreesToRadians(25.5)));

        _robotToSouthWestCam = new Transform3d(
                new Translation3d(-0.107009, 0.104835, 0.57991),
                new Rotation3d(0.0, 0.0, Units.degreesToRadians(-162.5)));

        _southEastCameraEstimator = new PhotonPoseEstimator(
                layout,
                PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                _southEastCamera,
                _robotToSouthEastCam);

        _northEastCameraEstimator = new PhotonPoseEstimator(
                layout,
                PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                _northEastCamera,
                _robotToNorthEastCam);

        _northWestCameraEstimator = new PhotonPoseEstimator(
                layout,
                PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                _northWestCamera,
                _robotToNorthWestCam);

        _southWestCameraEstimator = new PhotonPoseEstimator(
                layout,
                PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                _southWestCamera,
                _robotToSouthWestCam);

        _southEastCameraEstimator.setMultiTagFallbackStrategy(
                PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
        _northEastCameraEstimator.setMultiTagFallbackStrategy(
                PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
        _northWestCameraEstimator.setMultiTagFallbackStrategy(
                PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
        _southWestCameraEstimator.setMultiTagFallbackStrategy(
                PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
    }

    public void setPipeline(Pipeline pipeline) {
        _southEastCamera.setPipelineIndex(pipeline.getValue());
        _northEastCamera.setPipelineIndex(pipeline.getValue());
        _northWestCamera.setPipelineIndex(pipeline.getValue());
        _southWestCamera.setPipelineIndex(pipeline.getValue());
    }

    public double getSouthEastCameraLatency() {
        return _southEastCamera.getLatestResult().getLatencyMillis();
    }

    public double getNorthEastCameraLatency() {
        return _northEastCamera.getLatestResult().getLatencyMillis();
    }

    public double getNorthWestCameraLatency() {
        return _northWestCamera.getLatestResult().getLatencyMillis();
    }

    public double getSouthWestCameraLatency() {
        return _southWestCamera.getLatestResult().getLatencyMillis();
    }

    public boolean hasSouthEastCameraTargets() {
        return _southEastCamera.getLatestResult().hasTargets();
    }

    public boolean hasNorthEastCameraTargets() {
        return _northEastCamera.getLatestResult().hasTargets();
    }

    public boolean hasNorthWestCameraTargets() {
        return _northWestCamera.getLatestResult().hasTargets();
    }

    public boolean hasSouthWestCameraTargets() {
        return _southWestCamera.getLatestResult().hasTargets();
    }

    public boolean isSouthEastTargetsWithinAmbiguity(double ambiguityTolerance) {
        var tags = _southEastCamera.getLatestResult().targets;
        for (var tag : tags) {
            if (tag.getPoseAmbiguity() < ambiguityTolerance) {
                return false;
            }
        }
        return true;
    }

    public boolean isNorthEastTargetsWithinAmbiguity(double ambiguityTolerance) {
        var tags = _northEastCamera.getLatestResult().targets;
        for (var tag : tags) {
            if (tag.getPoseAmbiguity() < ambiguityTolerance) {
                return false;
            }
        }
        return true;
    }

    public boolean isNorthWestTargetsWithinAmbiguity(double ambiguityTolerance) {
        var tags = _northWestCamera.getLatestResult().targets;
        for (var tag : tags) {
            if (tag.getPoseAmbiguity() < ambiguityTolerance) {
                return false;
            }
        }
        return true;
    }

    public boolean isSouthWestTargetsWithinAmbiguity(double ambiguityTolerance) {
        var tags = _southWestCamera.getLatestResult().targets;
        for (var tag : tags) {
            if (tag.getPoseAmbiguity() < ambiguityTolerance) {
                return false;
            }
        }
        return true;
    }

    public void setLowestAmbiguity() {
        _southEastCameraEstimator.setPrimaryStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
        _northEastCameraEstimator.setPrimaryStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
        _northWestCameraEstimator.setPrimaryStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
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

    public Optional<EstimatedRobotPose> getNorthEastCameraEstimatedGlobalPose(
            Pose2d prevEstimatedPose) {
        _northEastCameraEstimator.setReferencePose(prevEstimatedPose);
        PhotonPipelineResult results = _northEastCamera.getLatestResult();
        if (results.hasTargets()) {
            results.targets.removeIf(tag -> tag.getPoseAmbiguity() > Constants.Vision.AMBIGUITY_TOLERANCE);
        }
        return _northEastCameraEstimator.update(results);
    }

    public Optional<EstimatedRobotPose> getNorthWestCameraEstimatedGlobalPose(
            Pose2d prevEstimatedPose) {
        _northWestCameraEstimator.setReferencePose(prevEstimatedPose);
        PhotonPipelineResult results = _northWestCamera.getLatestResult();
        if (results.hasTargets()) {
            results.targets.removeIf(tag -> tag.getPoseAmbiguity() > Constants.Vision.AMBIGUITY_TOLERANCE);
        }
        return _northWestCameraEstimator.update(results);
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

    public Pair<Optional<EstimatedRobotPose>, String> getNorthEastCameraEstimatedGlobalPoseWithName(
            Pose2d prevEstimatedPose) {
        return new Pair<Optional<EstimatedRobotPose>, String>(getNorthEastCameraEstimatedGlobalPose(prevEstimatedPose),
                "NorthEast");
    }

    public Pair<Optional<EstimatedRobotPose>, String> getNorthWestCameraEstimatedGlobalPoseWithName(
            Pose2d prevEstimatedPose) {
        return new Pair<Optional<EstimatedRobotPose>, String>(getNorthWestCameraEstimatedGlobalPose(prevEstimatedPose),
                "NorthWest");
    }

    public Pair<Optional<EstimatedRobotPose>, String> getSouthEastCameraEstimatedGlobalPoseWithName(
            Pose2d prevEstimatedPose) {
        return new Pair<Optional<EstimatedRobotPose>, String>(getSouthEastCameraEstimatedGlobalPose(prevEstimatedPose),
                "SouthEast");
    }

    public Pair<Optional<EstimatedRobotPose>, String> getSouthWestCameraEstimatedGlobalPoseWithName(
            Pose2d prevEstimatedPose) {
        return new Pair<Optional<EstimatedRobotPose>, String>(getSouthWestCameraEstimatedGlobalPose(prevEstimatedPose),
                "SouthWest");
    }

    public Optional<Double> calculateAngleToTag(int tagId) {
        PhotonPipelineResult southEastResults = _southEastCamera.getLatestResult();
        PhotonPipelineResult southWestResults = _southWestCamera.getLatestResult();
        
        Optional<Double> southEastAngle = Optional.empty();
        Optional<Double> southWestAngle = Optional.empty();
        
        // Always check if targets are available
        if (southEastResults.hasTargets()) {
            Optional<PhotonTrackedTarget> southEastTag = southEastResults.getTargets().stream()
                    .filter(target -> target.getFiducialId() == tagId)
                    .findFirst();
            
            if (southEastTag.isPresent()) {
                var camToTarget = southEastTag.get().getBestCameraToTarget();
                var robotToTarget = _robotToSouthEastCam.plus(camToTarget);
                southEastAngle = Optional.of(Math.atan2(robotToTarget.getY(), robotToTarget.getX()));
            }
        }
        
        // Always check if targets are available
        if (southWestResults.hasTargets()) {
            Optional<PhotonTrackedTarget> southWestTag = southWestResults.getTargets().stream()
                    .filter(target -> target.getFiducialId() == tagId)
                    .findFirst();
            
            if (southWestTag.isPresent()) {
                var camToTarget = southWestTag.get().getBestCameraToTarget();
                var robotToTarget = _robotToSouthWestCam.plus(camToTarget);
                southWestAngle = Optional.of(Math.atan2(robotToTarget.getY(), robotToTarget.getX()));
            }
        }
        
        if (southEastAngle.isPresent() && southWestAngle.isPresent()) {
            double averageAngle = (southEastAngle.get() + southWestAngle.get()) / 2.0;
            double adjustedAngle = Math.atan2(Math.sin(averageAngle), Math.cos(averageAngle));

            return Optional.of(adjustedAngle);
        } else if (southEastAngle.isPresent()) {
            double adjustedAngle = Math.atan2(Math.sin(southEastAngle.get()), Math.cos(southEastAngle.get()));

            return Optional.of(adjustedAngle);
        } else if (southWestAngle.isPresent()) {
            double adjustedAngle = Math.atan2(Math.sin(southWestAngle.get()), Math.cos(southWestAngle.get()));

            return Optional.of(adjustedAngle);
        } else {
            return Optional.empty();
        }
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
