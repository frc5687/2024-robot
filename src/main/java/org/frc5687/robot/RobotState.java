package org.frc5687.robot;

import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import javax.swing.text.html.Option;

import org.frc5687.Messages.VisionPose;
import org.frc5687.Messages.VisionPoseArray;
import org.frc5687.robot.subsystems.DriveTrain;
import org.frc5687.robot.util.PhotonProcessor;
import org.frc5687.robot.util.VisionProcessor;
import org.photonvision.EstimatedRobotPose;

import com.neilalexander.jnacl.crypto.poly1305;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;


public class RobotState {
    private DriveTrain _driveTrain;
    private PhotonProcessor _photonProcessor;
    private VisionProcessor _visionProcessor;
    private SwerveDrivePoseEstimator _poseEstimator;

    private static AprilTagFieldLayout _layout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    private static RobotState _instance;

    private Transform3d _robotToCamera;

    public static RobotState getInstance() {
        if (_instance == null) _instance = new RobotState();
        return _instance;
    }

    public RobotState() {}

    public void initializeRobotState(DriveTrain driveTrain, PhotonProcessor photonProcessor, VisionProcessor visionProcessor) {
        _driveTrain = driveTrain;
        _photonProcessor = photonProcessor;
        _visionProcessor = visionProcessor;
        _robotToCamera = new Transform3d(
            0.381, 0.0285, 0.3556, 
            new Rotation3d());
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
        if (measurement.toPose2d().getX() < Constants.FieldConstants.FIELD_WIDTH && measurement.toPose2d().getX() > 0
         && measurement.toPose2d().getY() < Constants.FieldConstants.FIELD_LENGTH && measurement.toPose2d().getY() > 0){
            return true;
        } else {
            return false;
        }
    }

    public Pose3d getSpeakerTagPose() {
        return _layout.getTagPose(
            DriverStation.getAlliance().get() == Alliance.Red ? 4 : 7
        ).get();
    }

    public Pair<Double, Double> getDistanceAndAngleToSpeaker() {
        Pose2d robotPose = getEstimatedPose();
        Pose3d tagPose = getSpeakerTagPose();

        double xDistance = tagPose.getX() - robotPose.getX();
        double yDistance = tagPose.getY() - robotPose.getY();

        double distance = Math.sqrt(
            Math.pow(xDistance, 2) + Math.pow(yDistance, 2)
        );

        double angle = Math.atan2(yDistance, xDistance) + Math.PI;

        // using a pair here to return both values without doing excess math in multiple methods
        return new Pair<Double, Double>(distance, angle);
    }

    // edit this as needed to reflect the optimal range to shoot from
    public boolean isWithinOptimalRange() {
        return getDistanceAndAngleToSpeaker().getFirst() < Constants.Shooter.OPTIMAL_SHOT_DISTANCE_THRESHOLD;
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
     * @param y     in meters of how much we trust y component
     * @param angle in radians of how much we trust the IMU;
     * @return Standard Deivation of the pose;
     */
    protected Vector<N3> createStateStandardDeviations(double x, double y, double angle) {
        return createStandardDeviations(x, y, angle);
    }

    protected Vector<N3> createVisionStandardDeviations(double x, double y, double angle) {
        return createStandardDeviations(x, y, angle);
    }

    public Optional<Pose2d> getClosestNote() {
        VisionPoseArray poses = _visionProcessor.getDetectedObjects();
        double closestDistance = Double.MAX_VALUE;
        Optional<Pose2d> closestNotePose = Optional.empty();

        for (int i = 0; i < poses.posesLength(); i++) {
            VisionPose pose = poses.poses(i);
            if (!Double.isNaN(pose.x()) && !Double.isNaN(pose.y())) {
                double distance = Math.hypot(pose.x(), pose.y());
                if (distance < closestDistance) {
                    closestDistance = distance;
                    closestNotePose = Optional.of(new Pose2d(pose.x(), pose.y(), new Rotation2d()));
                }
            }
        }

        return closestNotePose;
    }

    public Optional<Rotation2d> getAngleToClosestNote() {
        Optional<Pose2d> optionalPose = getClosestNote();
        if (optionalPose.isEmpty()) {
            return Optional.empty();
        }
        Pose2d pose = optionalPose.get();
        _driveTrain.readIMU();
        return Optional.of(Rotation2d.fromRadians(Math.atan2(pose.getY(), pose.getY())).minus(_driveTrain.getHeading()));
    }

    public Optional<Pose2d> getClosestNoteRelativeField() {
        Pose2d robotPose = getEstimatedPose(); 
        Optional<Pose2d> optionalPose = getClosestNote();
        if (optionalPose.isEmpty()) {
            return Optional.empty();
        }
        Pose2d notePoseRelativeRobot = optionalPose.get();
        
        Transform2d noteTransform = new Transform2d(notePoseRelativeRobot.getTranslation(), notePoseRelativeRobot.getRotation());
        Pose2d notePoseRelativeField = robotPose.transformBy(noteTransform);
        
        return Optional.of(notePoseRelativeField);
    }
}
