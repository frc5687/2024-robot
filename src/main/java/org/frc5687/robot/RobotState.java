package org.frc5687.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import org.frc5687.lib.cheesystuff.InterpolatingDouble;
import org.frc5687.robot.subsystems.DriveTrain;
import org.frc5687.robot.util.PhotonProcessor;
import org.frc5687.robot.util.VisionProcessor;
import org.frc5687.robot.util.VisionProcessor.DetectedNote;
import org.frc5687.robot.util.VisionProcessor.DetectedNoteArray;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

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
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Threads;

public class RobotState {

    public static class SwerveDriveState {
        public Pose2d pose;
        public ChassisSpeeds speeds;
    }

    //should probably reflect the layout in RobotContainer at some point.
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

    private volatile SwerveDriveState _cachedState = new SwerveDriveState();
    private volatile Pose2d _estimatedPose = new Pose2d();
    private volatile Twist2d _velocity = new Twist2d();
    private volatile Pose2d _lastPose = new Pose2d();

    private volatile Optional<Rotation2d> _visionAngle = Optional.empty();
    private volatile Optional<Double> _visionDistance = Optional.empty();

    // private volatile Pair<EstimatedRobotPose, String>[] _latestCameraPoses = new Pair[4];
    private volatile boolean _useVisionUpdates = true;

    private boolean _isAutoAiming = false;

    private static RobotState _instance;
    private double _lastTimestamp;
    private Transform3d _robotToCamera;
    private final double _period = 1.0 / 250.0; // Run at 200Hz

    private boolean[] _notesPickedUp = {false, false, false, false, false, false, false, false};

    public RobotState() {
    }

    public static RobotState getInstance() {
        if (_instance == null) {
            _instance = new RobotState();
        }
        return _instance;
    }

    public void initializeRobotState(DriveTrain driveTrain, PhotonProcessor photonProcessor,
            VisionProcessor visionProcessor) {
        _driveTrain = driveTrain;
        _photonProcessor = photonProcessor;
        _visionProcessor = visionProcessor;
        _lastTimestamp = Timer.getFPGATimestamp();

        _robotToCamera = new Transform3d(
            Constants.RobotState.ZED_X_OFFSET, 
            Constants.RobotState.ZED_Y_OFFSET, 
            Constants.RobotState.ZED_Z_OFFSET,
            new Rotation3d(
                Constants.RobotState.ZED_ROLL,
                Constants.RobotState.ZED_PITCH,
                Constants.RobotState.ZED_YAW
            )
        );
        initPoseEstimator();
        _periodicThread = new Thread(this::run);
        _periodicThread.setName("RobotState Thread");

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
                        Constants.VisionConfig.VISION_STD_DEV_X,
                        Constants.VisionConfig.VISION_STD_DEV_Y,
                        Constants.VisionConfig.VISION_STD_DEV_ANGLE));
    }

    private void run() {
        Threads.setCurrentThreadPriority(true, 1); // People say lowest priority works well. 
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
            // else {
            //     DriverStation.reportWarning("State thread exceeded "+_period+" second period. duration was "+duration+" seconds", false);
            // }
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
        _driveTrain.readSignals();

        SwerveModulePosition[] positions = _driveTrain.getSwerveModuleMeasuredPositions();
        Rotation2d heading = _driveTrain.getHeading();
        Pose2d currentPose = _poseEstimator.getEstimatedPosition();
        double currentTime = Timer.getFPGATimestamp();
        double deltaTime = currentTime - _lastTimestamp;
        ChassisSpeeds measured = _driveTrain.getMeasuredChassisSpeeds();
        _velocity = new Twist2d(measured.vxMetersPerSecond, measured.vyMetersPerSecond, measured.omegaRadiansPerSecond);

        if (deltaTime > 0) {
            _lastPose = new Pose2d(currentPose.getTranslation(), heading);
        }

        _poseEstimator.update(heading, positions);
        _lastTimestamp = currentTime;
        // _estimatedPose = _poseEstimator.getEstimatedPosition();
    }
    

    private void updateWithVision() {
        Pose2d prevEstimatedPose = _estimatedPose;
        List<Pair<EstimatedRobotPose, String>> cameraPoses = Stream.of(
                _photonProcessor.getSouthEastCameraEstimatedGlobalPoseWithName(prevEstimatedPose),
                _photonProcessor.getSouthCameraEstimatedGlobalPoseWithName(prevEstimatedPose),
                _photonProcessor.getSouthWestCameraEstimatedGlobalPoseWithName(prevEstimatedPose))
                .filter(pair -> pair.getFirst() != null)
                .filter(pair -> isValidMeasurementTest(pair))
                .collect(Collectors.toList());
            
        // for (int i = 0; i < cameraPoses.size() && i < 4; i++) {
        //     _latestCameraPoses[i] = cameraPoses.get(i);
        // }

       cameraPoses.forEach(this::processVisionMeasurement);
    }

    public void periodic() {
        updateOdometry();

        if (_useVisionUpdates) {
            updateWithVision();
        }

        // _visionAngle = getAngleToTagFromVision(getSpeakerTargetTagId());
        // _visionDistance = getDistanceToTagFromVision(getSpeakerTargetTagId());

        // if (_visionAngle.isPresent()) {
        //     SmartDashboard.putNumber("Vision Angle", _visionAngle.get().getRadians());
        // }
        // if (_visionDistance.isPresent()) {
        //     SmartDashboard.putNumber("Vision Distance", _visionDistance.get());
        // }
        _estimatedPose = _poseEstimator.getEstimatedPosition();
    }

    public synchronized Pose2d getEstimatedPose() {
        return _estimatedPose;
    }

    public synchronized void setEstimatedPose(Pose2d pose) {
        _poseEstimator.resetPosition(_driveTrain.getHeading(), _driveTrain.getSwerveModuleMeasuredPositions(), pose);
        _estimatedPose = pose;
        _lastPose = pose;
    }

    public void enableVisionUpdates() {
        _useVisionUpdates = true;
    }

    public void disableVisionUpdates() {
        _useVisionUpdates = false;
    }

    private boolean isValidMeasurementTest(Pair<EstimatedRobotPose, String> estimatedRobotPose) {
        Pose3d measurement = estimatedRobotPose.getFirst().estimatedPose;
        // PhotonTrackedTarget[] tagsUsed = estimatedRobotPose.targetsUsed.;

        // String cameraName = estimatedRobotPose.getSecond();
        if (measurement.getX() > Constants.FieldConstants.FIELD_LENGTH) {
            // DriverStation.reportError("According to " + cameraName +", Robot is off the
            // field in +x direction", false);
            return false;
        } else if (measurement.getX() < 0) {
            // DriverStation.reportError("According to " + cameraName +", Robot is off the
            // field in -x direction", false);
            return false;
        } else if (measurement.getY() > Constants.FieldConstants.FIELD_WIDTH) {
            // DriverStation.reportError("According to " + cameraName +", Robot is off the
            // field in +y direction", false);
            return false;
        } else if (measurement.getY() < 0) {
            // DriverStation.reportError("According to " + cameraName +", Robot is off the
            // field in the -y direction", false);
            return false;
        } else if (measurement.getZ() < -0.15) {
            // DriverStation.reportError("According to " + cameraName +", Robot is inside
            // the floor :(((", false);
            return false;
        } else if (measurement.getZ() > 0.15) {
            // DriverStation.reportError("According to " + cameraName +", Robot is floating
            // above the floor :(((", false);
            return false;
        }
        return true;
    }

    public int getSpeakerTargetTagId() {
        return _driveTrain.isRedAlliance() ? 4 : 7;
    }

    public Pose3d getSpeakerOpeningPose() {
        Pose3d tagToAim = _layout.getTagPose(getSpeakerTargetTagId()).get();
        double offset = isRedAlliance() ? -0.1 : 0.1;
        return new Pose3d(tagToAim.getX() + offset, tagToAim.getY(), tagToAim.getZ(), tagToAim.getRotation());
    }

    public Pair<Double, Double> getDistanceAndAngleToSpeaker() {
        Pose2d robotPose = getEstimatedPose();
        Pose3d tagPose = getSpeakerOpeningPose();

        double xDistance = tagPose.getX() - robotPose.getX();
        double yDistance = tagPose.getY() - robotPose.getY();

        double distance = Math.sqrt(
                Math.pow(xDistance, 2) + Math.pow(yDistance, 2));

        // flip because intake is pi radians from shooter
        Rotation2d angle = new Rotation2d(Math.atan2(yDistance, xDistance)).plus(new Rotation2d(Math.PI));

        // using a pair here to return both values without doing excess math in multiple
        // methods
        return new Pair<Double, Double>(distance, angle.getRadians());
    }

    public Pair<Double, Double> getDistanceAndAngleToCorner() {
        Pose2d robotPose = getEstimatedPose();
        Pose2d cornerPose = _driveTrain.isRedAlliance() ? Constants.RobotState.RED_CORNER : Constants.RobotState.BLUE_CORNER;

        double xDistance = cornerPose.getX() - robotPose.getX();
        double yDistance = cornerPose.getY() - robotPose.getY();

        double distance = Math.sqrt(
                Math.pow(xDistance, 2) + Math.pow(yDistance, 2));

        // flip because intake is pi radians from shooter
        Rotation2d angle = new Rotation2d(Math.atan2(yDistance, xDistance)).plus(new Rotation2d(Math.PI));

        // using a pair here to return both values without doing excess math in multiple
        // methods
        return new Pair<Double, Double>(distance, angle.getRadians());
    }

    public Pair<Double, Double> calculateAdjustedRPMAndAngleToTarget() {
        Pair<Double, Double> initialDistanceAndAngle = getDistanceAndAngleToSpeaker();
        double initialDistance = initialDistanceAndAngle.getFirst();

        double initialShooterRPM = Constants.Shooter.kRPMMap
                .getInterpolated(new InterpolatingDouble(initialDistance)).value;

        double shotTravelTime = calculateShotTravelTime(initialDistance, initialShooterRPM);
        double futureX = _lastPose.getX() + _velocity.dx * shotTravelTime;
        double futureY = _lastPose.getY() + _velocity.dy * shotTravelTime;

        Pose2d futurePose = new Pose2d(futureX, futureY, _driveTrain.getHeading());
        Pose3d targetPose = getSpeakerOpeningPose();
        // This tries to predict the RPM you want while moving, Didn't seem to help much so we just used the initial guess RPM.
        // double futureDistance = Math.hypot(targetPose.getX() - futurePose.getX(),
        //         targetPose.getY() - futurePose.getY());

        // double adjustedShooterRPM = Constants.Shooter.kRPMMap
        //         .getInterpolated(new InterpolatingDouble(futureDistance)).value;

        Rotation2d adjustedAngle = new Rotation2d(
                Math.atan2(targetPose.getY() - futurePose.getY(), targetPose.getX() - futurePose.getX()));

        return new Pair<>(initialShooterRPM, adjustedAngle.getRadians());
    }

    public Pose2d calculateAdjustedRPMAndAngleToTargetPose() {
        Pair<Double, Double> initialDistanceAndAngle = getDistanceAndAngleToSpeaker();
        double initialDistance = initialDistanceAndAngle.getFirst();

        double initialShooterRPM = Constants.Shooter.kRPMMap
                .getInterpolated(new InterpolatingDouble(initialDistance)).value;

        double shotTravelTime = calculateShotTravelTime(initialDistance, initialShooterRPM);
        double futureX = _lastPose.getX() + _velocity.dx * shotTravelTime;
        double futureY = _lastPose.getY() + _velocity.dy * shotTravelTime;

        Pose2d futurePose = new Pose2d(futureX, futureY, _driveTrain.getHeading());
        Pose3d targetPose = getSpeakerOpeningPose();

        Rotation2d adjustedAngle = new Rotation2d(
                Math.atan2(targetPose.getY() - futurePose.getY(), targetPose.getX() - futurePose.getX()));
        return new Pose2d(futureX, futureY, adjustedAngle);
    }

    public Pose2d predictedPositionWithVelocity(double shootTime) {
        double futureX = _lastPose.getX() + _velocity.dx * shootTime;
        double futureY = _lastPose.getY() + _velocity.dy * shootTime;

        Pose2d futurePose = new Pose2d(futureX, futureY, _driveTrain.getHeading());
        Pose3d targetPose = getSpeakerOpeningPose();
        Rotation2d adjustedAngle = new Rotation2d(
                Math.atan2(targetPose.getY() - futurePose.getY(), targetPose.getX() - futurePose.getX()));
        futurePose = new Pose2d(futureX, futureY, adjustedAngle);
        return futurePose;
    }

    private double calculateShotTravelTime(double distance, double shooterRPM) {
        double wheelCircumference = Math.PI * Constants.Shooter.WHEEL_DIAMETER_METERS;
        double wheelRPS = (shooterRPM / Constants.Shooter.GEAR_RATIO) / 60.0;
        double linearVelocity = wheelRPS * wheelCircumference;

        return distance / linearVelocity;
        // return 0.05;
    }

    // edit this as needed to reflect the optimal range to shoot from
    public boolean isWithinOptimalRange() {
        double distance = getDistanceAndAngleToSpeaker().getFirst();
        return distance > Constants.Shooter.OPTIMAL_SHOT_DISTANCE_LOWER_LIMIT
                && distance < Constants.Shooter.OPTIMAL_SHOT_DISTANCE_UPPER_LIMIT;
    }

    public boolean isAimedAtSpeaker() {
        Optional<Boolean> visionAimed = isVisionAimedAtSpeaker();

        if (visionAimed.isPresent()) {
            return visionAimed.get();
        }

        return isPoseAimedAtSpeaker();
    }

    private Optional<Boolean> isVisionAimedAtSpeaker() {
        Optional<Rotation2d> visionAngle = getAngleToSpeakerFromVision();
        if (visionAngle.isPresent()) {
            return Optional.of(Math.abs(_driveTrain.getHeading().minus(visionAngle.get()).getRadians()) > Constants.RobotState.VISION_AIMING_TOLERANCE);
        }
        return Optional.empty();
    }

    private boolean isPoseAimedAtSpeaker() {
        Rotation2d heading = _driveTrain.getHeading();
        Rotation2d targetAngle = new Rotation2d(getDistanceAndAngleToSpeaker().getSecond());

        // i chose a function w a period of pi bc i didn't wanna deal w if red side was +pi.... worlds 2024, xavier
        double lerpInputValue = Math.abs(targetAngle.getSin());
        double tolerance = Constants.DriveTrain.ANGLED_HEADING_TOLERANCE * lerpInputValue + Constants.DriveTrain.STRAIGHT_HEADING_TOLERANCE * (1 - lerpInputValue);

        Rotation2d difference = heading.minus(targetAngle);
        // SmartDashboard.putNumber("Speaker angle error", difference.getRadians());
        // SmartDashboard.putNumber("Tolerance", tolerance);
        return Math.abs(difference.getRadians()) < tolerance;
    }

    public boolean isAimedAtCorner() {
        Optional<Boolean> visionAimed = isVisionAimedAtCorner();

        if (visionAimed.isPresent()) {
            return visionAimed.get();
        }

        return isPoseAimedAtCorner();
    }

    private Optional<Boolean> isVisionAimedAtCorner() {
        // we should make getAngleToCornerFromVision at some point, hacking for now.
        // Optional<Rotation2d> visionAngle = getAngleToCornerFromVision();
        // if (visionAngle.isPresent()) {
        //     return Optional.of(Math.abs((_driveTrain.getHeading().minus(visionAngle.get()).getRadians())) > Constants.RobotState.VISION_AIMING_TOLERANCE);
        // }
        return Optional.empty();
    }

    private boolean isPoseAimedAtCorner() {
        Rotation2d heading = _driveTrain.getHeading();
        Rotation2d targetAngle = new Rotation2d(getDistanceAndAngleToCorner().getSecond());

        Rotation2d difference = heading.minus(targetAngle);
        return Math.abs(difference.getRadians()) < Constants.DriveTrain.PASS_HEADING_TOLERANCE;
    }

    /**
     * breaks if the driver station has no alliance
     * 
     * @param pose the pose to check if it is on the alliance side :o
     * @return boolean idk
     */
    public boolean isOnAllianceHalf(Pose2d pose) {
        if (getAlliance().get() == Alliance.Red) {
            return pose.getX() > Constants.FieldConstants.FIELD_LENGTH / 2.0;
        } else {
            return pose.getX() < Constants.FieldConstants.FIELD_LENGTH / 2.0;
        }
    }

    public boolean crossedTheMidline(Pose2d pose) {
        if (getAlliance().get() == Alliance.Red) {
            return pose.getX() < Constants.FieldConstants.FIELD_LENGTH / 2.0 - 0.3; // i aint putting this in constants no way
        } else {
            return pose.getX() > Constants.FieldConstants.FIELD_LENGTH / 2.0 + 0.3;
        }
    }

    public Optional<Alliance> getAlliance() {
        return DriverStation.getAlliance();
    }

    public boolean isRedAlliance() {
        return _driveTrain.isRedAlliance();
    }

    private void processVisionMeasurement(Pair<EstimatedRobotPose, String> cameraPose) {
        EstimatedRobotPose estimatedPose = cameraPose.getFirst();

        double dist = estimatedPose.estimatedPose.toPose2d().getTranslation().getDistance(_estimatedPose.getTranslation());
        
        double positionDev, angleDev;

        // DriverStation.reportError(estimatedPose.strategy.name(), false);
        if (estimatedPose.strategy == PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR) {
            // multi-tag estimate, trust it more
            positionDev = 0.1;
            angleDev = Units.degreesToRadians(10); // Try this but IMU is still probably way better
        } else {
            // single-tag estimates, adjust deviations based on distance
            if (dist < 1.5) {
                positionDev = 0.30;
                angleDev = Units.degreesToRadians(500);
            } else if (dist < 4.0) {
                positionDev = 0.45;
                angleDev = Units.degreesToRadians(500);
            } else {
                positionDev = 0.5;
                angleDev = Units.degreesToRadians(500);
            }
        }
        
        _poseEstimator.setVisionMeasurementStdDevs(
            createVisionStandardDeviations(positionDev, positionDev, angleDev)
        );
        
        _poseEstimator.addVisionMeasurement(estimatedPose.estimatedPose.toPose2d(),
                estimatedPose.timestampSeconds + Constants.RobotState.VISION_TIMESTAMP_FUDGE);
    }
   

    // public void useAutoStandardDeviations() {
    //     _poseEstimator.setVisionMeasurementStdDevs(createVisionStandardDeviations(
    //             Constants.VisionConfig.Auto.VISION_STD_DEV_X,
    //             Constants.VisionConfig.Auto.VISION_STD_DEV_Y,
    //             Constants.VisionConfig.Auto.VISION_STD_DEV_ANGLE));
    // }

    // public void useTeleopStandardDeviations() {
    //     _poseEstimator.setVisionMeasurementStdDevs(createVisionStandardDeviations(
    //             Constants.VisionConfig.Teleop.VISION_STD_DEV_X,
    //             Constants.VisionConfig.Teleop.VISION_STD_DEV_Y,
    //             Constants.VisionConfig.Teleop.VISION_STD_DEV_ANGLE));
    // }

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

    private Optional<Double> getDistanceToTagFromVision(int tagId) {
        Optional<Double> distance = _photonProcessor.calculateDistanceToTag(tagId);
        return distance;
    }

    public Optional<Double> getDistanceToSpeakerFromVision() {
        return _visionDistance;
    }

    private Optional<Rotation2d> getAngleToTagFromVision(int tagId) {
        return _photonProcessor.calculateAngleToTag(tagId);
    }

    public Optional<Rotation2d> getAngleToSpeakerFromVision() {
        return _visionAngle;
    }

    public Optional<Pose3d> getClosestNoteRelativeRobotCenter() {
        DetectedNoteArray notes = _visionProcessor.getDetectedObjects();
        double closestDistance = Double.MAX_VALUE;
        Optional<Pose3d> closestNotePose = Optional.empty();

        for (DetectedNote note : notes.getNotes()) {
            Pose3d notePoseRelativeCamera = note.getPose();
            Pose3d notePoseRelativeRobotCenter = notePoseRelativeCamera.transformBy(_robotToCamera);
            Translation3d translation = notePoseRelativeRobotCenter.getTranslation();
            double distance = Math.sqrt(Math.pow(translation.getX(), 2) + Math.pow(translation.getY(), 2));
            if (distance < closestDistance) {
                closestDistance = distance;
                closestNotePose = Optional.of(notePoseRelativeRobotCenter);
            }
        }
        return closestNotePose;
    }

    /**
     * 
     * @param blindingRadians the angle outside which to reject note poses. For example, a value of 0.1 radians would reject notes at 0.11 radians and -0.11 radians
     * @return an optional pose of the closest note (within the blinded zone)
     */
    public Optional<Pose3d> getClosestNoteRelativeRobotCenterBlindedAngle(double blindingRadians) {
        DetectedNoteArray notes = _visionProcessor.getDetectedObjects();
        double closestDistance = Double.MAX_VALUE;
        Optional<Pose3d> closestNotePose = Optional.empty();

        for (DetectedNote note : notes.getNotes()) {
            Pose3d notePoseRelativeCamera = note.getPose();
            Pose3d notePoseRelativeRobotCenter = notePoseRelativeCamera.transformBy(_robotToCamera);
            Translation3d translation = notePoseRelativeRobotCenter.getTranslation();
            double noteAngleRadians = Math.atan2(translation.getY(), translation.getX());
            double distance = Math.sqrt(Math.pow(translation.getX(), 2) + Math.pow(translation.getY(), 2));
            // System.out.println("Note angle: "+noteAngleRadians+" radians, distance: "+distance+" meters");
            // reject notes outside the blinders 🐴
            if (noteAngleRadians > blindingRadians || noteAngleRadians < -blindingRadians) {
                continue;
            }
            if (distance < closestDistance) {
                closestDistance = distance;
                closestNotePose = Optional.of(notePoseRelativeRobotCenter);
            }
        }
        return closestNotePose;
    }

    public Optional<Pose2d> getClosestNoteRelativeField() {
        Pose2d robotPose = getEstimatedPose();
        Optional<Pose3d> optionalPose3d = getClosestNoteRelativeRobotCenter();
        if (optionalPose3d.isEmpty()) {
            return Optional.empty();
        }
        Pose3d notePoseRelativeRobotCenter = optionalPose3d.get();
        Pose2d notePose2dRelativeRobotCenter = notePoseRelativeRobotCenter.toPose2d();
        Transform2d noteTransform = new Transform2d(notePose2dRelativeRobotCenter.getTranslation(),
                notePose2dRelativeRobotCenter.getRotation());
        Pose2d notePoseRelativeField = robotPose.transformBy(noteTransform);

        double angleToNote = Math.atan2(notePoseRelativeField.getY() - robotPose.getY(),
                notePoseRelativeField.getX() - robotPose.getX());

        Rotation2d desiredRotation = new Rotation2d(angleToNote);

        return Optional.of(new Pose2d(notePoseRelativeField.getTranslation(), desiredRotation));
    }

    public Optional<List<Pose2d>> getAllNotesRelativeField() {
        Pose2d robotPose = getEstimatedPose();
        DetectedNoteArray notes = _visionProcessor.getDetectedObjects();
        List<Pose2d> notePosesRelativeField = new ArrayList<>();

        for (DetectedNote note : notes.getNotes()) {
            Pose3d notePoseRelativeCamera = note.getPose();
            Pose3d notePoseRelativeRobotCenter = notePoseRelativeCamera.transformBy(_robotToCamera);
            Pose2d notePose2dRelativeRobotCenter = notePoseRelativeRobotCenter.toPose2d();
            Transform2d noteTransform = new Transform2d(notePose2dRelativeRobotCenter.getTranslation(),
                    notePose2dRelativeRobotCenter.getRotation());
            Pose2d notePoseRelativeField = robotPose.transformBy(noteTransform);

            double angleToNote = Math.atan2(notePoseRelativeField.getY() - robotPose.getY(),
                    notePoseRelativeField.getX() - robotPose.getX());

            Rotation2d desiredRotation = new Rotation2d(angleToNote);

            notePosesRelativeField.add(new Pose2d(notePoseRelativeField.getTranslation(), desiredRotation));
        }

        if (notePosesRelativeField.isEmpty()) {
            return Optional.empty();
        }

        return Optional.of(notePosesRelativeField);
    }

    public ChassisSpeeds getMeasuredSpeeds() {
       return _driveTrain.getMeasuredChassisSpeeds();
    }

    public SwerveDriveState getState() {
        return _cachedState;
    }

    public Lock getWriteLock() {
        return writeLock;
    }

    public Lock getReadLock() {
        return readLock;
    }

    // id is from 1 - 8 not 0-7
    public void setIntakedNote(int id, boolean noteIntaked) {
        if (id - 1 >= _notesPickedUp.length) {
            DriverStation.reportError("Cannot set id greater than size of array", false);
            return;
        }
        _notesPickedUp[id - 1] = noteIntaked;
    }

    public boolean isNoteIntaked(int id) {
        if (id - 1 >= _notesPickedUp.length) {
            DriverStation.reportError("Cannot set id greater than size of array", false);
            return false;
        }
        return _notesPickedUp[id - 1];
    }

    public void setAutoAiming(boolean value) {
        _isAutoAiming = value;
    }

    public boolean isAutoAiming() {
        return _isAutoAiming;
    }

    // public Pair<EstimatedRobotPose, String>[] getLatestCameraPoses() {
    //     return _latestCameraPoses;
    // }
}