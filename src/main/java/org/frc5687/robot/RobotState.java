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
import org.frc5687.lib.cheesystuff.InterpolatingDouble;
import org.frc5687.robot.subsystems.DriveTrain;
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
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotState {
//     private static AprilTagFieldLayout _layout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    private Thread _periodicThread;
    private volatile boolean _running = false;

    private final ReadWriteLock stateLock = new ReentrantReadWriteLock();
    private final Lock readLock = stateLock.readLock();
    private final Lock writeLock = stateLock.writeLock();

    private DriveTrain _driveTrain;
//     private VisionProcessor _visionProcessor;
    private SwerveDrivePoseEstimator _poseEstimator;

    private Twist2d _velocity = new Twist2d();
    private Pose2d _lastPose = new Pose2d(); // To store the last pose for velocity calculation


    private static RobotState _instance;
    private double _lastTimestamp;
    private final double _period = 1.0 / 200.0; // Run at 200Hz

    public RobotState() {}

    public static RobotState getInstance() {
        if (_instance == null) {
            _instance = new RobotState();
        }
        return _instance;
    }

    public void initializeRobotState(DriveTrain driveTrain,  VisionProcessor visionProcessor) {
        _driveTrain = driveTrain;
        // _photonProcessor = photonProcessor;
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
            _driveTrain.readSignals();
            SwerveModulePosition[] positions = _driveTrain.getSwerveModuleMeasuredPositions();
            Rotation2d heading = _driveTrain.getHeading();
            Pose2d currentPose = _poseEstimator.getEstimatedPosition();
            double currentTime = Timer.getFPGATimestamp();
            double deltaTime = currentTime - _lastTimestamp;
            ChassisSpeeds measured = _driveTrain.getMeasuredChassisSpeeds();
            _velocity = new Twist2d(measured.vxMetersPerSecond, measured.vyMetersPerSecond, measured.omegaRadiansPerSecond);

            if (deltaTime > 0) {
                // Translation2d deltaPose = _lastPose.getTranslation().minus(currentPose.getTranslation());
                // Translation2d linearVelocity = deltaPose.div(deltaTime);

                // double deltaHeading = _lastPose.getRotation().minus(heading).getRadians();
                // double angularVelocity = deltaHeading / deltaTime;

                // _velocity = new Twist2d(linearVelocity.getX(), linearVelocity.getY(), angularVelocity);
                // _velocity = _velocityPredictor.getEstimatedVelocity();

                _lastPose = new Pose2d(currentPose.getTranslation(), heading);
            }

            _poseEstimator.update(heading, positions);
            _lastTimestamp = currentTime;
        } finally {
            writeLock.unlock();
        }
    }

//     private void updateWithVision() {
//         writeLock.lock();
//         try {
//             Pose2d prevEstimatedPose = getEstimatedPoseThreadSafe();

//             List<Pair<EstimatedRobotPose, String>> cameraPoses = Stream.of(
//                 _photonProcessor.getSouthEastCameraEstimatedGlobalPoseWithName(prevEstimatedPose),
//                 _photonProcessor.getNorthEastCameraEstimatedGlobalPoseWithName(prevEstimatedPose),
//                 _photonProcessor.getNorthWestCameraEstimatedGlobalPoseWithName(prevEstimatedPose),
//                 _photonProcessor.getSouthWestCameraEstimatedGlobalPoseWithName(prevEstimatedPose))
//             .map(pair -> new Pair<>(pair.getFirst().stream(), pair.getSecond())) 
//             .flatMap(pair -> pair.getFirst().map(estimate -> new Pair<>(estimate, pair.getSecond()))) // Handle Stream<Optional<EstimatedRobotPose>>
//             .filter(pair -> isValidMeasurementTest(pair))
//             .collect(Collectors.toList());
        
//             cameraPoses.forEach(cameraPose -> {
//                 // dynamicallyChangeDeviations(cameraPose.getFirst().estimatedPose, prevEstimatedPose);
//                 _poseEstimator.addVisionMeasurement(cameraPose.getFirst().estimatedPose.toPose2d(), cameraPose.getFirst().timestampSeconds);
//             });

//             // List<EstimatedRobotPose> cameraPoses = Stream.of(
//             //         _photonProcessor.getSouthEastCameraEstimatedGlobalPose(prevEstimatedPose),
//             //         _photonProcessor.getNorthEastCameraEstimatedGlobalPose(prevEstimatedPose),
//             //         _photonProcessor.getNorthWestCameraEstimatedGlobalPose(prevEstimatedPose),
//             //         _photonProcessor.getSouthWestCameraEstimatedGlobalPose(prevEstimatedPose))
//             //         .flatMap(Optional::stream)
//             //         .filter(cameraPose -> isValidMeasurement(cameraPose))
//             //         .collect(Collectors.toList());
    
//             // cameraPoses.forEach(cameraPose -> {
//             //     // dynamicallyChangeDeviations(cameraPose.estimatedPose, prevEstimatedPose);
//             //     _poseEstimator.addVisionMeasurement(cameraPose.estimatedPose.toPose2d(), cameraPose.timestampSeconds);
//             // });
//         } finally {
//             writeLock.unlock();
//         }
//     }

    public void periodic() {
        updateOdometry();
        // updateWithVision();

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
//     private boolean isValidMeasurementTest(Pair<EstimatedRobotPose, String> estimatedRobotPose) {
//         Pose3d measurement = estimatedRobotPose.getFirst().estimatedPose;
//         // PhotonTrackedTarget[] tagsUsed = estimatedRobotPose.targetsUsed.;

//         String cameraName = estimatedRobotPose.getSecond();
//         if (measurement.getX() > Constants.FieldConstants.FIELD_LENGTH) {
//             // DriverStation.reportError("According to " + cameraName +", Robot is off the field in +x direction", false);
//             return false;
//         } else if (measurement.getX() < 0) {
//             // DriverStation.reportError("According to " + cameraName +", Robot is off the field in -x direction", false);
//             return false;
//         } else if (measurement.getY() > Constants.FieldConstants.FIELD_WIDTH) {
//             // DriverStation.reportError("According to " + cameraName +", Robot is off the field in +y direction", false);
//             return false;
//         } else if (measurement.getY() < 0) {
//             // DriverStation.reportError("According to " + cameraName +", Robot is off the field in the -y direction", false);
//             return false;
//         } else if (measurement.getZ() < -0.15) {
//             // DriverStation.reportError("According to " + cameraName +", Robot is inside the floor :(((", false);
//             return false;
//         } else if (measurement.getZ() > 0.15) {
//             // DriverStation.reportError("According to " + cameraName +", Robot is floating above the floor :(((", false);
//             return false;
//         }
//         return true;
//     }

//     private boolean isValidMeasurement(EstimatedRobotPose estimatedRobotPose) {
//         Pose3d measurement = estimatedRobotPose.estimatedPose;
//         // PhotonTrackedTarget[] tagsUsed = estimatedRobotPose.targetsUsed.;

//         String cameraName = "none";
//         if (measurement.getX() > Constants.FieldConstants.FIELD_LENGTH) {
//             // DriverStation.reportError("According to " + cameraName +", Robot is off the field in +x direction", false);
//             return false;
//         } else if (measurement.getX() < 0) {
//             // DriverStation.reportError("According to " + cameraName +", Robot is off the field in -x direction", false);
//             return false;
//         } else if (measurement.getY() > Constants.FieldConstants.FIELD_WIDTH) {
//             // DriverStation.reportError("According to " + cameraName +", Robot is off the field in +y direction", false);
//             return false;
//         } else if (measurement.getY() < 0) {
//             // DriverStation.reportError("According to " + cameraName +", Robot is off the field in the -y direction", false);
//             return false;
//         } else if (measurement.getZ() < -0.15) {
//             // DriverStation.reportError("According to " + cameraName +", Robot is inside the floor :(((", false);
//             return false;
//         } else if (measurement.getZ() > 0.15) {
//             // DriverStation.reportError("According to " + cameraName +", Robot is floating above the floor :(((", false);
//             return false;
//         }
//         return true;
//     }

//     public Pose3d getSpeakerTagPose() {
//         return _layout.getTagPose(
//             _driveTrain.isRedAlliance() ? 4 : 7
//         ).get();
//     }

//     public Pair<Double, Double> getDistanceAndAngleToSpeaker() {
//         Pose2d robotPose = getEstimatedPose();
//         Pose3d tagPose = getSpeakerTagPose();

//         double xDistance = tagPose.getX() - robotPose.getX();
//         double yDistance = tagPose.getY() - robotPose.getY();

//         double distance = Math.sqrt(
//             Math.pow(xDistance, 2) + Math.pow(yDistance, 2)
//         );

//         // flip because intake is pi radians from shooter
//         Rotation2d angle = new Rotation2d(Math.atan2(yDistance, xDistance)).plus(new Rotation2d(Math.PI));

//         // using a pair here to return both values without doing excess math in multiple methods
//         return new Pair<Double, Double>(distance, angle.getRadians());
//     }

//     // Return a pair of <Shooter RPM, AngleToTarget>
//     public Pair<Double, Double> calculateAdjustedRPMAndAngleToTarget() {
//         Pair<Double, Double> initialDistanceAndAngle = getDistanceAndAngleToSpeaker();
//         double initialDistance = initialDistanceAndAngle.getFirst();
    
//         double initialShooterRPM = Constants.Shooter.kRPMMap.getInterpolated(new InterpolatingDouble(initialDistance)).value;
    
//         double shotTravelTime = calculateShotTravelTime(initialDistance, initialShooterRPM); 
//         double futureX = _lastPose.getX() + _velocity.dx * shotTravelTime;
//         double futureY = _lastPose.getY() + _velocity.dy * shotTravelTime;
    
//         Pose2d futurePose = new Pose2d(futureX, futureY, _driveTrain.getHeading());
//         Pose3d targetPose = getSpeakerTagPose();
//         double futureDistance = Math.hypot(targetPose.getX() - futurePose.getX(), targetPose.getY() - futurePose.getY());
    
//         double adjustedShooterRPM = Constants.Shooter.kRPMMap.getInterpolated(new InterpolatingDouble(futureDistance)).value; 
    
//         Rotation2d adjustedAngle = new Rotation2d(Math.atan2(targetPose.getY() - futurePose.getY(), targetPose.getX() - futurePose.getX()));
    
//         return new Pair<>(initialShooterRPM, adjustedAngle.getRadians());
//         // return new Pair<>(adjustedShooterRPM, adjustedAngle.getRadians());
//     }

//         // Return a pair of <Shooter RPM, AngleToTarget>
//         public Pose2d calculateAdjustedRPMAndAngleToTargetPose() {
//             Pair<Double, Double> initialDistanceAndAngle = getDistanceAndAngleToSpeaker();
//             double initialDistance = initialDistanceAndAngle.getFirst();
        
//             double initialShooterRPM = Constants.Shooter.kRPMMap.getInterpolated(new InterpolatingDouble(initialDistance)).value;
        
//             double shotTravelTime = calculateShotTravelTime(initialDistance, initialShooterRPM); 
//             double futureX = _lastPose.getX() + _velocity.dx * shotTravelTime;
//             double futureY = _lastPose.getY() + _velocity.dy * shotTravelTime;
        
//             Pose2d futurePose = new Pose2d(futureX, futureY, _driveTrain.getHeading());
//             Pose3d targetPose = getSpeakerTagPose();
        
//             Rotation2d adjustedAngle = new Rotation2d(Math.atan2(targetPose.getY() - futurePose.getY(), targetPose.getX() - futurePose.getX()));
//             return new Pose2d(futureX, futureY, adjustedAngle);
//         }

//     public Pose2d predictedPositionWithVelocity(double shootTime) {
//         double futureX = _lastPose.getX() + _velocity.dx * shootTime;
//         double futureY = _lastPose.getY() + _velocity.dy * shootTime;
    
//         Pose2d futurePose = new Pose2d(futureX, futureY,_driveTrain.getHeading());
//         Pose3d targetPose = getSpeakerTagPose();
//         Rotation2d adjustedAngle = new Rotation2d(Math.atan2(targetPose.getY() - futurePose.getY(), targetPose.getX() - futurePose.getX()));
//         futurePose = new Pose2d(futureX, futureY, adjustedAngle);
//         return futurePose;
//     }

//     private double calculateShotTravelTime(double distance, double shooterRPM) {
//         double wheelCircumference = Math.PI * Constants.Shooter.WHEEL_DIAMETER_METERS;
//         double wheelRPS = (shooterRPM / Constants.Shooter.GEAR_RATIO) / 60.0; 
//         double linearVelocity = wheelRPS * wheelCircumference;

//         return distance / linearVelocity;
//         // return 0.05;
//     }

//     // edit this as needed to reflect the optimal range to shoot from
//     public boolean isWithinOptimalRange() {
//         double distance = getDistanceAndAngleToSpeaker().getFirst();
//         return 
//             distance > Constants.Shooter.OPTIMAL_SHOT_DISTANCE_LOWER_LIMIT
//             && distance < Constants.Shooter.OPTIMAL_SHOT_DISTANCE_UPPER_LIMIT
//         ;
//     }

//     /**
//      * This changes the standard deviations to trust vision measurements less the
//      * farther the machine is.
//      * the linear line y = 0.13x + 0.3
//      * 
//      * @param measurement the measurement from an AprilTag
//      */
//     public void dynamicallyChangeDeviations(Pose3d measurement, Pose2d currentEstimatedPose) {
//         double dist = measurement.toPose2d().getTranslation().getDistance(currentEstimatedPose.getTranslation());
//         double positionDev = Math.abs(0.2 * dist + 0.2);
//         _poseEstimator.setVisionMeasurementStdDevs(
//                 createVisionStandardDeviations(positionDev, positionDev, Units.degreesToRadians(400)));
//     }

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

//     /**
//      * @param x     in meters of how much we trust x component
//      * @param y     in meters of how much we trust y component
//      * @param angle in radians of how much we trust the IMU;
//      * @return Standard Deivation of the pose;
//      */
    protected Vector<N3> createStateStandardDeviations(double x, double y, double angle) {
        return createStandardDeviations(x, y, angle);
    }

    protected Vector<N3> createVisionStandardDeviations(double x, double y, double angle) {
        return createStandardDeviations(x, y, angle);
    }

//     public Optional<Pose2d> getClosestNote() {
//         VisionPoseArray poses = _visionProcessor.getDetectedObjects();
//         double closestDistance = Double.MAX_VALUE;
//         Optional<Pose2d> closestNotePose = Optional.empty();

//         for (int i = 0; i < poses.posesLength(); i++) {
//             VisionPose pose = poses.poses(i);
//             if (!Double.isNaN(pose.x()) && !Double.isNaN(pose.y())) {
//                 double distance = Math.hypot(pose.x(), pose.y());
//                 if (distance < closestDistance) {
//                     closestDistance = distance;
//                     closestNotePose = Optional.of(new Pose2d(pose.x(), pose.y(), new Rotation2d()));
//                 }
//             }
//         }
//         return closestNotePose;
//     }

//     public Optional<Rotation2d> getAngleToClosestNote() {
//         Optional<Pose2d> optionalPose = getClosestNote();
//         if (optionalPose.isEmpty()) {
//             return Optional.empty();
//         }
//         Pose2d pose = optionalPose.get();
//         _driveTrain.readIMU();
//         return Optional.of(Rotation2d.fromRadians(Math.atan2(pose.getY(), pose.getY())).minus(_driveTrain.getHeading()));
//     }

//     public Optional<Pose2d> getClosestNoteRelativeField() {
//         Pose2d robotPose = getEstimatedPose(); 
//         Optional<Pose2d> optionalPose = getClosestNote();
//         if (optionalPose.isEmpty()) {
//             return Optional.empty();
//         }
//         Pose2d notePoseRelativeRobot = optionalPose.get();
        
//         Transform2d noteTransform = new Transform2d(notePoseRelativeRobot.getTranslation(), notePoseRelativeRobot.getRotation());
//         Pose2d notePoseRelativeField = robotPose.transformBy(noteTransform);
        
//         return Optional.of(notePoseRelativeField);
//     }
    public Lock getWriteLock() {
        return writeLock;
    }

    public Lock getReadLock() {
        return readLock;
    }
}