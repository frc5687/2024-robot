package org.frc5687.robot;

import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import org.frc5687.robot.subsystems.DriveTrain.DriveTrain;
import org.frc5687.robot.util.PhotonProcessor;
import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public class RobotState {
    private DriveTrain _driveTrain;
    private PhotonProcessor _photonProcessor;

    private SwerveDrivePoseEstimator _poseEstimator;

    public RobotState(DriveTrain driveTrain, PhotonProcessor photonProcessor) {
        _driveTrain = driveTrain;
        _photonProcessor = photonProcessor;

        initPoseEstimator();
    }

    private void initPoseEstimator() {
        _poseEstimator = new SwerveDrivePoseEstimator(
            _driveTrain.getKinematics(),
            _driveTrain.getHeading(),
            _driveTrain.getSwerveModuleMeasuredPositions(),
            new Pose2d(0, 0, _driveTrain.getHeading()),
            createStateStandardDeviations(
                Constants.VisionConfig.STATE_STD_DEV_X,
                Constants.VisionConfig.STATE_STD_DEV_Y,
                Constants.VisionConfig.STATE_STD_DEV_ANGLE),
            createVisionStandardDeviations(
                        Constants.VisionConfig.VISION_STD_DEV_X,
                        Constants.VisionConfig.VISION_STD_DEV_Y,
                        Constants.VisionConfig.VISION_STD_DEV_ANGLE)
            );
    }

    public void updateOdometry() {
        SwerveModulePosition[] positions = _driveTrain.getSwerveModuleMeasuredPositions();
        Rotation2d heading = _driveTrain.getHeading();
        _poseEstimator.update(heading, positions);
    }

    public void updateWithVision() {
        Pose2d prevEstimatedPose = _poseEstimator.getEstimatedPosition();
        List<EstimatedRobotPose> cameraPoses = Stream.of(
                _photonProcessor.getSouthEastCameraEstimatedGlobalPose(prevEstimatedPose),
                _photonProcessor.getNorthEastCameraEstimatedGlobalPose(prevEstimatedPose),
                _photonProcessor.getNorthWestCameraEstimatedGlobalPose(prevEstimatedPose),
                _photonProcessor.getSouthWestCameraEstimatedGlobalPose(prevEstimatedPose))
                .flatMap(Optional::stream)
                .filter(cameraPose -> isValidMeasurement(cameraPose.estimatedPose))
                .collect(Collectors.toList());

        cameraPoses.forEach(cameraPose -> {
            dynamicallyChangeDeviations(cameraPose.estimatedPose, prevEstimatedPose);
            _poseEstimator.addVisionMeasurement(
                    cameraPose.estimatedPose.toPose2d(),
                    cameraPose.timestampSeconds);
        });
    }

    public void periodic() {
        updateOdometry();
        updateWithVision();
    }

    public Pose2d getEstimatedPose() {
        return _poseEstimator.getEstimatedPosition();
    }

    private boolean isValidMeasurement(Pose3d measurement) {
        return true;
    }


    /**
     * This changes the standard deviations to trust vision measurements less the
     * farther the machine is.
     * the linear line y = 0.13x + 0.3
     * 
     * @param measurement the measurement from an AprilTag
     */
    public void dynamicallyChangeDeviations(Pose3d measurement, Pose2d currentEstimatedPose) {
        double dist = measurement.toPose2d().getTranslation().getDistance(currentEstimatedPose.getTranslation());
        double positionDev = Math.abs(0.2 * dist + 0.2);
        _poseEstimator.setVisionMeasurementStdDevs(
                createVisionStandardDeviations(positionDev, positionDev, Units.degreesToRadians(400)));
    }

    protected Vector<N3> createStandardDeviations(double x, double y, double z) {
        return VecBuilder.fill(x, y, z);
    }

    /**
     * @param x     in meters of how much we trust x component
     * @param y     in meters of how much we trust x component
     * @param angle in radians of how much we trust the IMU;
     * @return Standard Deivation of the pose;
     */
    protected Vector<N3> createStateStandardDeviations(double x, double y, double angle) {
        return createStandardDeviations(x, y, angle);
    }

    protected Vector<N3> createVisionStandardDeviations(double x, double y, double angle) {
        return createStandardDeviations(x, y, angle);
    }

    
}
