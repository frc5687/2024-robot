package org.frc5687.robot.util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;

import java.util.Optional;
import java.util.concurrent.CompletableFuture;
//import java.util.concurrent.ExecutorService;
//import java.util.concurrent.Executors;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

public class PhotonProcessor {

    //private static final int NUM_CAMERAS = 4;
    private final PhotonCamera _northCamera;
    private final PhotonCamera _southCamera;
    private final PhotonCamera _westCamera;
    private final PhotonCamera _eastCamera;
    private final PhotonPoseEstimator _northCameraEstimator;
    private final PhotonPoseEstimator _southCameraEstimator;
    private final PhotonPoseEstimator _westCameraEstimator;
    private final PhotonPoseEstimator _eastCameraEstimator;

    //private final ExecutorService _executorService;
    public PhotonProcessor(AprilTagFieldLayout layout) {
        _northCamera = new PhotonCamera("North_Camera"); 
        _southCamera = new PhotonCamera("South_Camera");
        _westCamera = new PhotonCamera("West_Camera");
        _eastCamera = new PhotonCamera("East_Camera");
        //_executorService = Executors.newFixedThreadPool(NUM_CAMERAS);

        setPipeline(Pipeline.FAR);
        // new values taken from z to floor
        // old values
        Transform3d robotToNorthCam =
            new Transform3d(
                new Translation3d(0.08, 0.09, 0.62),
                new Rotation3d(0, 0, 0)
            );
        Transform3d robotToSouthCam =
            new Transform3d(
                new Translation3d(0.00, 0.09, 0.57), 
                new Rotation3d(0, 0, Math.PI)
            );

        Transform3d robotToWestCam =
            new Transform3d(
                new Translation3d(0.04, 0.12, 0.52),
                new Rotation3d(0, 0, 0.5 * Math.PI)
            );

        Transform3d robotToEastCam =
            new Transform3d(
                new Translation3d(0.04, 0.05, 0.47),
                new Rotation3d(0, 0, 1.5 * Math.PI)
            );

        _northCameraEstimator =
            new PhotonPoseEstimator(
                layout,
                PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                _northCamera,
                robotToNorthCam);

        _southCameraEstimator =
            new PhotonPoseEstimator(
                layout,
                PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                _southCamera,
                robotToSouthCam);

        _westCameraEstimator =
            new PhotonPoseEstimator(
                layout,
                PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                _westCamera,
                robotToWestCam);

        _eastCameraEstimator =
                new PhotonPoseEstimator(
                        layout,
                        PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                        _eastCamera,
                        robotToEastCam);

        _northCameraEstimator.setMultiTagFallbackStrategy(
                PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
        _southCameraEstimator.setMultiTagFallbackStrategy(
                PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
        _westCameraEstimator.setMultiTagFallbackStrategy(
                PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
        _eastCameraEstimator.setMultiTagFallbackStrategy(
                PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
    }

    public void setPipeline(Pipeline pipeline) {
        _northCamera.setPipelineIndex(pipeline.getValue());
        _southCamera.setPipelineIndex(pipeline.getValue());
        _westCamera.setPipelineIndex(pipeline.getValue());
        _eastCamera.setPipelineIndex(pipeline.getValue());
    }

    public double getNorthCameraLatency() {
        return _northCamera.getLatestResult().getLatencyMillis();
    }

    public double getSouthCameraLatency() {
        return _southCamera.getLatestResult().getLatencyMillis();
    }

    public boolean hasNorthCameraTargets() {
        return _northCamera.getLatestResult().hasTargets();
    }

    public boolean hasSouthCameraTargets() {
        return _southCamera.getLatestResult().hasTargets();
    }

    public void setLowestAmbiguity() {
        _northCameraEstimator.setPrimaryStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
        _southCameraEstimator.setPrimaryStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
        _westCameraEstimator.setPrimaryStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
        _eastCameraEstimator.setPrimaryStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
    }

    public Optional<EstimatedRobotPose> getNorthCameraEstimatedGlobalPose(
            Pose2d prevEstimatedPose) {
        _northCameraEstimator.setReferencePose(prevEstimatedPose);
        return _northCameraEstimator.update();
    }

    public Optional<EstimatedRobotPose> getSouthCameraEstimatedGlobalPose(
            Pose2d prevEstimatedPose) {
        _southCameraEstimator.setReferencePose(prevEstimatedPose);
        return _southCameraEstimator.update();
    }

    public Optional<EstimatedRobotPose> getWestCameraEstimatedGlobalPose(
            Pose2d prevEstimatedPose) {
        _westCameraEstimator.setReferencePose(prevEstimatedPose);
        return _westCameraEstimator.update();
    }

    public Optional<EstimatedRobotPose> getEastCameraEstimatedGlobalPose(
            Pose2d prevEstimatedPose) {
        _eastCameraEstimator.setReferencePose(prevEstimatedPose);
        return _eastCameraEstimator.update();
    }

    public CompletableFuture<Optional<EstimatedRobotPose>> getNorthCameraEstimatedGlobalPoseAsync(
            Pose2d prevEstimatedPose) {
        return CompletableFuture.supplyAsync(
                () -> getNorthCameraEstimatedGlobalPose(prevEstimatedPose));
    }

    public CompletableFuture<Optional<EstimatedRobotPose>> getSouthTopCameraEstimatedGlobalPoseAsync(
            Pose2d prevEstimatedPose) {
        return CompletableFuture.supplyAsync(
                () -> getSouthCameraEstimatedGlobalPose(prevEstimatedPose));
    }

    public CompletableFuture<Optional<EstimatedRobotPose>> getWestCameraEstimatedGlobalPoseAsync(
                Pose2d prevEstimatedPose) {
        return CompletableFuture.supplyAsync(
                () -> getWestCameraEstimatedGlobalPose(prevEstimatedPose));
    }

    public CompletableFuture<Optional<EstimatedRobotPose>> getEastCameraEstimatedGlobalPoseAsync(
            Pose2d prevEstimatedPose) {
        return CompletableFuture.supplyAsync(
                () -> getEastCameraEstimatedGlobalPose(prevEstimatedPose));
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

