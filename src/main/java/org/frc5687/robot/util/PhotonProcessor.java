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

    public PhotonProcessor(AprilTagFieldLayout layout) {
        _southEastCamera = new PhotonCamera("South_East_Camera");
        _northEastCamera = new PhotonCamera("North_East_Camera");
        _northWestCamera = new PhotonCamera("North_West_Camera");
        _southWestCamera = new PhotonCamera("South_West_Camera");

        // FIXME: look at the order the rotation transformations are applied -xavier
        // bradford
        Transform3d robotToSouthEastCam = new Transform3d(
                // new Translation3d(0.0, -0.155635, 0.5937), // for centered camera
                new Translation3d(-0.107009, -0.104835, 0.57991),
                // new Rotation3d(0.0, 0.0, Units.degreesToRadians(180)) // for centered camera
                new Rotation3d(0.0, 0.0, Units.degreesToRadians(162.5)));

        Transform3d robotToNorthEastCam = new Transform3d(
                new Translation3d(Units.inchesToMeters(3.90), Units.inchesToMeters(-7.05), Units.inchesToMeters(11.00)),
                new Rotation3d(0.0, Units.degreesToRadians(16.5), Units.degreesToRadians(-25.5)));

        Transform3d robotToNorthWestCam = new Transform3d(
                new Translation3d(Units.inchesToMeters(3.90), Units.inchesToMeters(7.05), Units.inchesToMeters(11.00)),
                new Rotation3d(0.0, Units.degreesToRadians(16.5), Units.degreesToRadians(25.5)));

        Transform3d robotToSouthWestCam = new Transform3d(
                new Translation3d(-0.107009, 0.104835, 0.57991),
                new Rotation3d(0.0, 0.0, Units.degreesToRadians(-162.5)));

        _southEastCameraEstimator = new PhotonPoseEstimator(
                layout,
                PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                _southEastCamera,
                robotToSouthEastCam);

        _northEastCameraEstimator = new PhotonPoseEstimator(
                layout,
                PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                _northEastCamera,
                robotToNorthEastCam);

        _northWestCameraEstimator = new PhotonPoseEstimator(
                layout,
                PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                _northWestCamera,
                robotToNorthWestCam);

        _southWestCameraEstimator = new PhotonPoseEstimator(
                layout,
                PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                _southWestCamera,
                robotToSouthWestCam);

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
    
        Optional<PhotonTrackedTarget> southEastTag = Optional.empty();
        Optional<PhotonTrackedTarget> southWestTag = Optional.empty();
    
        if (southEastResults.hasTargets()) {
            southEastTag = southEastResults.getTargets().stream()
                    .filter(target -> target.getFiducialId() == tagId)
                    .findFirst();
        }
    
        if (southWestResults.hasTargets()) {
            southWestTag = southWestResults.getTargets().stream()
                    .filter(target -> target.getFiducialId() == tagId)
                    .findFirst();
        }
    
        if (southEastTag.isPresent() && southWestTag.isPresent()) {
            var southEastPose = southEastTag.get().getBestCameraToTarget().plus(_southEastCameraEstimator.getRobotToCameraTransform().inverse());
            var southWestPose = southWestTag.get().getBestCameraToTarget().plus(_southWestCameraEstimator.getRobotToCameraTransform().inverse());
        
            Translation2d southEastTranslation = southEastPose.getTranslation().toTranslation2d();
            Translation2d southWestTranslation = southWestPose.getTranslation().toTranslation2d();
        
            Translation2d deltaTranslation = southWestTranslation.minus(southEastTranslation);
            Rotation2d angle = new Rotation2d(deltaTranslation.getX(), deltaTranslation.getY());
        
            return Optional.of(angle.getDegrees());
        } else if (southEastTag.isPresent()) {
            var southEastPose = southEastTag.get().getBestCameraToTarget().plus(_southEastCameraEstimator.getRobotToCameraTransform().inverse());
            Rotation3d rotation = southEastPose.getRotation();
            double angleRadians = rotation.getZ();
            return Optional.of(angleRadians);
        } else if (southWestTag.isPresent()) {
            var southWestPose = southWestTag.get().getBestCameraToTarget().plus(_southWestCameraEstimator.getRobotToCameraTransform().inverse());
            Rotation3d rotation = southWestPose.getRotation();
            double angleRadians = rotation.getZ();
            return Optional.of(angleRadians);
        }
        
        return Optional.empty();
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
