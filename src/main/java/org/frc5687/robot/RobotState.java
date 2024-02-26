package org.frc5687.robot;

import java.util.List;
import java.util.Optional;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import org.frc5687.Messages.VisionPose;
import org.frc5687.Messages.VisionPoseArray;
import org.frc5687.robot.subsystems.DriveTrain;
import org.frc5687.robot.util.PhotonProcessor;
import org.frc5687.robot.util.VisionProcessor;
import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotState {
    private static AprilTagFieldLayout _layout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    private Thread _periodicThread;
    private volatile boolean _running = false;

    private final ReadWriteLock stateLock = new ReentrantReadWriteLock();
    private final Lock readLock = stateLock.readLock();
    private final Lock writeLock = stateLock.writeLock();

    private DriveTrain _driveTrain;
    private PhotonProcessor _photonProcessor;
    private VisionProcessor _visionProcessor;
    private SwerveDrivePoseEstimator _poseEstimator;
    private Translation2d _velocity = new Translation2d();
    private Pose2d _lastPose = new Pose2d(); // To store the last pose for velocity calculation


    private static RobotState _instance;
    private double _lastTimestamp;
    private final double _period = 1.0 / 250.0; // Run at 250Hz

    public RobotState() {

    }

    public static RobotState getInstance() {
        if (_instance == null) {
            _instance = new RobotState();
        }
        return _instance;
    }

    public void initializeRobotState(DriveTrain driveTrain, PhotonProcessor photonProcessor, VisionProcessor visionProcessor) {
        _driveTrain = driveTrain;
        _photonProcessor = photonProcessor;
        _visionProcessor = visionProcessor;
        _lastTimestamp = Timer.getFPGATimestamp();
        // _robotToCamera = new Transform3d(
        //     0.381, 0.0285, 0.3556, 
        //     new Rotation3d());
        initPoseEstimator();
        _periodicThread = new Thread(this::run);
        _periodicThread.setDaemon(true);
        _lastTimestamp = Timer.getFPGATimestamp();
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
                        Constants.VisionConfig.Auto.VISION_STD_DEV_X,
                        Constants.VisionConfig.Auto.VISION_STD_DEV_Y,
                        Constants.VisionConfig.Auto.VISION_STD_DEV_ANGLE)
            );
    }

    private void run() {
        _running = true;
        while (_running) {
            double startTime = Timer.getFPGATimestamp();
            
            periodic();
            
            double endTime = Timer.getFPGATimestamp();
            double duration = endTime - startTime;
            double sleepTime = _period - duration;
            
            if (sleepTime > 0) {
                Timer.delay(sleepTime);
            }
        }
    }

    public void start() {
        if (!_periodicThread.isAlive()) {
            _periodicThread.start();
        }
    }

    public void stop() {
        _running = false;
        try {
            _periodicThread.join();
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    private void updateOdometry() {
        writeLock.lock();
        try {
            SwerveModulePosition[] positions = _driveTrain.getSwerveModuleMeasuredPositions();
            Rotation2d heading = _driveTrain.getHeading();
            Pose2d currentPose = _poseEstimator.getEstimatedPosition();
            double currentTime = Timer.getFPGATimestamp();
            double deltaTime = currentTime - _lastTimestamp;
            if (deltaTime > 0) {
                Translation2d deltaPose =_lastPose.getTranslation().minus(currentPose.getTranslation());
                _velocity = deltaPose.div(deltaTime);
                _lastPose = new Pose2d(currentPose.getTranslation(), new Rotation2d());
            }
            _poseEstimator.update(heading, positions);
            _lastTimestamp = currentTime;
        } finally {
            writeLock.unlock();
        }
    }

    private void updateWithVision() {
        writeLock.lock();
        try {
            Pose2d prevEstimatedPose = getEstimatedPoseThreadSafe();
            List<EstimatedRobotPose> cameraPoses = Stream.of(
                    _photonProcessor.getSouthEastCameraEstimatedGlobalPose(prevEstimatedPose),
                    _photonProcessor.getNorthEastCameraEstimatedGlobalPose(prevEstimatedPose),
                    _photonProcessor.getNorthWestCameraEstimatedGlobalPose(prevEstimatedPose),
                    _photonProcessor.getSouthWestCameraEstimatedGlobalPose(prevEstimatedPose))
                    .flatMap(Optional::stream)
                    .filter(cameraPose -> isValidMeasurement(cameraPose.estimatedPose))
                    .collect(Collectors.toList());
    
            cameraPoses.forEach(cameraPose -> {
                // dynamicallyChangeDeviations(cameraPose.estimatedPose, prevEstimatedPose);
                _poseEstimator.addVisionMeasurement(cameraPose.estimatedPose.toPose2d(), cameraPose.timestampSeconds);
            });
        } finally {
            writeLock.unlock();
        }
    }

    public void periodic() {
        updateOdometry();
        updateWithVision();

        readLock.lock();
        try {
            SmartDashboard.putNumber("Estimated X", getEstimatedPose().getX());
            SmartDashboard.putNumber("Estimated Y", getEstimatedPose().getY());
        } finally {
            readLock.unlock();
        }
    }

    private Pose2d getEstimatedPoseThreadSafe() {
        readLock.lock();
        try {
            Pose2d pose = _poseEstimator.getEstimatedPosition();
            return pose;
        } finally {
            readLock.unlock();
        }
    }
    
    public Pose2d getEstimatedPose() {
        return getEstimatedPoseThreadSafe(); //  thread-safe (probably)
    }

    public void setEstimatedPose(Pose2d pose) {
        // FIXME: these values might not be right
        _poseEstimator.resetPosition(_driveTrain.getHeading(), _driveTrain.getSwerveModuleMeasuredPositions(), pose);
    }

    private boolean isValidMeasurement(Pose3d measurement) {
        if (measurement.getX() > Constants.FieldConstants.FIELD_LENGTH) {
            DriverStation.reportError("Robot is off the field in +x direction", false);
            return false;
        } else if (measurement.getX() < 0) {
            DriverStation.reportError("Robot is off the field in -x direction", false);
            return false;
        } else if (measurement.getY() > Constants.FieldConstants.FIELD_WIDTH) {
            DriverStation.reportError("Robot is off the field in +y direction", false);
            return false;
        } else if (measurement.getY() < 0) {
            DriverStation.reportError("Robot is off the field in the -y direction", false);
            return false;
        } else if (measurement.getZ() < -0.15) {
            DriverStation.reportError("Robot is inside the floor :(((", false);
            return false;
        } else if (measurement.getZ() > 0.15) {
            DriverStation.reportError("Robot is floating above the floor :(((", false);
            return false;
        }
        return true;
    }

    public Pose3d getSpeakerTagPose() {
        return _layout.getTagPose(
            _driveTrain.isRedAlliance() ? 4 : 7
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

        // flip because intake is pi radians from shooter
        Rotation2d angle = new Rotation2d(Math.atan2(yDistance, xDistance)).plus(new Rotation2d(Math.PI));

        // using a pair here to return both values without doing excess math in multiple methods
        return new Pair<Double, Double>(distance, angle.getRadians());
    }

    public Pair<Double, Double> calculateAdjustedAngleToTarget(double shooterRPM) {
        Pair<Double, Double> distanceAndAngleToTarget = getDistanceAndAngleToSpeaker();
        double shotTravelTime = calculateShotTravelTime(distanceAndAngleToTarget.getFirst(), shooterRPM);
        
        double futureX = _lastPose.getX() + _velocity.getX() * shotTravelTime;
        double futureY = _lastPose.getY() + _velocity.getY() * shotTravelTime;
        Pose2d futurePose = new Pose2d(futureX, futureY, _driveTrain.getHeading());

        Pose3d targetPose = getSpeakerTagPose();
        double adjustedXDistance = targetPose.getX() - futurePose.getX();
        double adjustedYDistance = targetPose.getY() - futurePose.getY();
        
        double adjustedDistance = Math.hypot(adjustedXDistance, adjustedYDistance);
        Rotation2d adjustedAngle = new Rotation2d(Math.atan2(adjustedYDistance, adjustedXDistance));

        return new Pair<>(adjustedDistance, adjustedAngle.getRadians());
    }

    private double calculateShotTravelTime(double distance, double outputVelocity) {
        return distance / getInitialVelocity(outputVelocity);
    }

    private double getInitialVelocity(double flywheelMapRPM) {
        double flywheelRadius = 0.1; 

        double flywheelAngularVelocity = flywheelMapRPM* (2 * Math.PI / 60);
        double initialVelocity = flywheelAngularVelocity * flywheelRadius;

        return initialVelocity; 
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

    public void useAutoStandardDeviations() {
        _poseEstimator.setVisionMeasurementStdDevs(createVisionStandardDeviations(
            Constants.VisionConfig.Auto.VISION_STD_DEV_X,
            Constants.VisionConfig.Auto.VISION_STD_DEV_Y,
            Constants.VisionConfig.Auto.VISION_STD_DEV_ANGLE));
    }

    public void useTeleopStandardDeviations() {
        _poseEstimator.setVisionMeasurementStdDevs(createVisionStandardDeviations(
            Constants.VisionConfig.Teleop.VISION_STD_DEV_X,
            Constants.VisionConfig.Teleop.VISION_STD_DEV_Y,
            Constants.VisionConfig.Teleop.VISION_STD_DEV_ANGLE));
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