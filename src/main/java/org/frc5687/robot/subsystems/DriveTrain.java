/* Team 5687 (C)2020-2022 */
package org.frc5687.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
// import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static org.frc5687.robot.Constants.DriveTrain.*;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import org.frc5687.lib.control.SwerveHeadingController;
import org.frc5687.lib.control.SwerveHeadingController.HeadingState;
import org.frc5687.lib.swerve.SwerveSetpoint;
import org.frc5687.lib.swerve.SwerveSetpointGenerator;
import org.frc5687.lib.swerve.SwerveSetpointGenerator.KinematicLimits;
import org.frc5687.lib.vision.TrackedObjectInfo;
import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotMap;
import org.frc5687.robot.util.*;
// import org.frc5687.lib.vision.VisionProcessor;
import org.photonvision.EstimatedRobotPose;

public class DriveTrain extends OutliersSubsystem {
    // Order we define swerve modules in kinematics
    private static final int NORTH_WEST_IDX = 0;
    private static final int SOUTH_WEST_IDX = 1;
    private static final int SOUTH_EAST_IDX = 2;
    private static final int NORTH_EAST_IDX = 3;
    private final DoubleSolenoid _shift;
    private final PneumaticHub _pneumaticHub;
    private final Compressor _compressor;
    private final SwerveModule[] _modules;
    private final SwerveDriveKinematics _kinematics;
    private final SwerveDriveOdometry _odometry;
    private final SwerveDrivePoseEstimator _poseEstimator;
    // module sensor readings we would like to read all at once over CAN
    private final BaseStatusSignal[] _moduleSignals;
    // controllers [Heading, Pose, Trajectory]
    private ControlState _controlState;
    private final SwerveHeadingController _headingController;
    private final HolonomicDriveController _poseController;
    private final PPHolonomicDriveController _trajectoryController;

    // IMU (Pigeon)
    private final Pigeon2 _imu;
    private double _yawOffset;

    private boolean _isLowGear;

    // Setpoint generator for swerve.
    private final SwerveSetpointGenerator _swerveSetpointGenerator;
    private KinematicLimits _kinematicLimits = LOW_KINEMATIC_LIMITS;

    private final SystemIO _systemIO;

    // Vision Processors
    // private final VisionProcessor _visionProcessor;
    private final PhotonProcessor _photonProcessor;

    private final Field2d _field;
    private Mode _mode = Mode.NORMAL;
    private Pose2d _hoverGoal;

    private boolean _shiftLockout = false;
    private long _shiftTime = 0;

    private Pose2d _wantedRestPose;
    private boolean _wantsToSetPose = false;

    private boolean _hasShiftInit = false;

    private boolean _hasPressure = false;

    public DriveTrain(
            OutliersContainer container,
            // VisionProcessor processor,
            PhotonProcessor photonProcessor,
            Pigeon2 imu) {
        super(container);

        // _visionProcessor = processor;
        _photonProcessor = photonProcessor;

        _shift = new DoubleSolenoid(2, PneumaticsModuleType.REVPH,
                RobotMap.PCM.SHIFTER_HIGH, RobotMap.PCM.SHIFTER_LOW);
        _pneumaticHub = new PneumaticHub(2);
        // _pneumaticHub.makeDoubleSolenoid(RobotMap.PCM.SHIFTER_HIGH, RobotMap.PCM.SHIFTER_LOW);
        _compressor = new Compressor(2, PneumaticsModuleType.REVPH);
        _compressor.enableDigital();
        // _pneumaticHub.enableCompressorDigital();

        // configure our system IO and pigeon;
        _imu = imu;
        _systemIO = new SystemIO();

        // This should set the Pigeon to 0.
        // set update frequency to 200hz
        _imu.getYaw().setUpdateFrequency(200);
        _imu.getPitch().setUpdateFrequency(200);
        _yawOffset = _imu.getYaw().getValue();
        readIMU();

        _controlState = ControlState.NEUTRAL;

        // set up the modules
        _modules = new SwerveModule[4];
        _modules[NORTH_WEST_IDX] = new SwerveModule(
                Constants.DriveTrain.NORTH_WEST_CONFIG,
                RobotMap.CAN.TALONFX.NORTH_WEST_ROTATION,
                RobotMap.CAN.TALONFX.NORTH_WEST_TRANSLATION,
                RobotMap.CAN.CANCODER.ENCODER_NW);
        _modules[SOUTH_WEST_IDX] = new SwerveModule(
                Constants.DriveTrain.SOUTH_WEST_CONFIG,
                RobotMap.CAN.TALONFX.SOUTH_WEST_ROTATION,
                RobotMap.CAN.TALONFX.SOUTH_WEST_TRANSLATION,
                RobotMap.CAN.CANCODER.ENCODER_SW);
        _modules[SOUTH_EAST_IDX] = new SwerveModule(
                Constants.DriveTrain.SOUTH_EAST_CONFIG,
                RobotMap.CAN.TALONFX.SOUTH_EAST_ROTATION,
                RobotMap.CAN.TALONFX.SOUTH_EAST_TRANSLATION,
                RobotMap.CAN.CANCODER.ENCODER_SE);
        _modules[NORTH_EAST_IDX] = new SwerveModule(
                Constants.DriveTrain.NORTH_EAST_CONFIG,
                RobotMap.CAN.TALONFX.NORTH_EAST_ROTATION,
                RobotMap.CAN.TALONFX.NORTH_EAST_TRANSLATION,
                RobotMap.CAN.CANCODER.ENCODER_NE);

        // This should set the Pigeon to 0.
        _imu.getYaw().setUpdateFrequency(200);
        _imu.getPitch().setUpdateFrequency(200);
        _yawOffset = _imu.getYaw().getValue();
        readIMU();

        _controlState = ControlState.MANUAL;

        _isLowGear = true;
        _kinematics = new SwerveDriveKinematics(
                _modules[NORTH_WEST_IDX].getModuleLocation(),
                _modules[SOUTH_WEST_IDX].getModuleLocation(),
                _modules[SOUTH_EAST_IDX].getModuleLocation(),
                _modules[NORTH_EAST_IDX].getModuleLocation());
        _odometry = new SwerveDriveOdometry(
                _kinematics,
                getHeading(),
                new SwerveModulePosition[] {
                        _modules[NORTH_WEST_IDX].getModulePosition(),
                        _modules[SOUTH_WEST_IDX].getModulePosition(),
                        _modules[SOUTH_EAST_IDX].getModulePosition(),
                        _modules[NORTH_EAST_IDX].getModulePosition()
                },
                new Pose2d(0, 0, getHeading()));

        _poseEstimator = new SwerveDrivePoseEstimator(
                _kinematics,
                getHeading(),
                new SwerveModulePosition[] {
                        _modules[NORTH_WEST_IDX].getModulePosition(),
                        _modules[SOUTH_WEST_IDX].getModulePosition(),
                        _modules[SOUTH_EAST_IDX].getModulePosition(),
                        _modules[NORTH_EAST_IDX].getModulePosition()
                },
                new Pose2d(0, 0, getHeading()),
                createStateStandardDeviations(
                        Constants.VisionConfig.STATE_STD_DEV_X,
                        Constants.VisionConfig.STATE_STD_DEV_Y,
                        Constants.VisionConfig.STATE_STD_DEV_ANGLE),
                createVisionStandardDeviations(
                        Constants.VisionConfig.VISION_STD_DEV_X,
                        Constants.VisionConfig.VISION_STD_DEV_Y,
                        Constants.VisionConfig.VISION_STD_DEV_ANGLE));
        _swerveSetpointGenerator = new SwerveSetpointGenerator(
                _kinematics,
                new Translation2d[] {
                        _modules[NORTH_WEST_IDX].getModuleLocation(),
                        _modules[SOUTH_WEST_IDX].getModuleLocation(),
                        _modules[SOUTH_EAST_IDX].getModuleLocation(),
                        _modules[NORTH_EAST_IDX].getModuleLocation()
                });

        // controllers
        _headingController = new SwerveHeadingController(Constants.UPDATE_PERIOD);

        _poseController = new HolonomicDriveController(
                new PIDController(
                        Constants.DriveTrain.kP, Constants.DriveTrain.kI, Constants.DriveTrain.kD),
                new PIDController(
                        Constants.DriveTrain.kP, Constants.DriveTrain.kI, Constants.DriveTrain.kD),
                new ProfiledPIDController(
                        MAINTAIN_kP,
                        MAINTAIN_kI,
                        MAINTAIN_kD,
                        new TrapezoidProfile.Constraints(
                                Constants.DriveTrain.PROFILE_CONSTRAINT_VEL,
                                Constants.DriveTrain.PROFILE_CONSTRAINT_ACCEL)));

        _trajectoryController = new PPHolonomicDriveController(
            new PIDConstants(0,0,0),
            new PIDConstants(0,0,0),
            0,0,0);

        // module CAN bus sensor outputs (position, velocity of each motor) all of them
        // are called once per loop at the start.
        _moduleSignals = new BaseStatusSignal[NUM_MODULES * 4];
        for (int i = 0; i < NUM_MODULES; ++i) {
            var signals = _modules[i].getSignals();
            _moduleSignals[(i * 4)] = signals[0];
            _moduleSignals[(i * 4) + 1] = signals[1];
            _moduleSignals[(i * 4) + 2] = signals[2];
            _moduleSignals[(i * 4) + 3] = signals[3];
        }

        _field = new Field2d();
        _hoverGoal = new Pose2d();

        zeroGyroscope();
        resetRobotPose(new Pose2d());

        readModules();
        setSetpointFromMeasuredModules();
        logMetrics("Drivetrain Speed",
                "NW Current", "NE Current", "SW Current", "SE Current"/*
                                                                       * ,
                                                                       * "NW Velocity Wanted", "NE Velocity Wanted",
                                                                       * "SW Velocity Wanted", "SE Velocity Wanted",
                                                                       * "NW Velocity", "NE Velocity", "SW Velocity",
                                                                       * "SE Velocity"
                                                                       */);
    }

    /**
     * SystemIO is input, output of our drivetrain that we want to cache.
     */
    public static class SystemIO {
        ChassisSpeeds desiredChassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
        SwerveModuleState[] measuredStates = new SwerveModuleState[] {
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState()
        };
        SwerveModulePosition[] measuredPositions = new SwerveModulePosition[] {
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition()
        };
        Rotation2d heading = new Rotation2d(0.0);
        double pitch = 0.0;

        Pose2d estimatedPose = new Pose2d();
        // outputs
        SwerveSetpoint setpoint = new SwerveSetpoint(new ChassisSpeeds(), new SwerveModuleState[4]);
    }

    // DriveTrain periodic functions, these run constantly even if there is no
    // command scheduled or the robot is disabled.
    // public void modulePeriodic() {
    // // use for modules as controller is running at 200Hz.
    // BaseStatusSignal.waitForAll(0.0, _moduleSignals);
    // for (SwerveModule SwerveModule : _modules) {
    // SwerveModule.controlPeriodic();
    // }
    // }
    @Override
    public void periodic() {
        super.periodic();
        // _pneumaticHub.disableCompressor();
        // _pneumaticHub.enableCompressorDigital();
        // _compressor.get
        if (!_compressor.getPressureSwitchValue()){
            _compressor.disable();
        } 
        if (!_hasShiftInit) {
            shiftDownModules();
            _hasShiftInit = true;
        }
        BaseStatusSignal.waitForAll(0.0, _moduleSignals);
        readIMU();
        readModules();
        updateDesiredStates();
        setModuleStates(_systemIO.setpoint.moduleStates);
    }

    // Heading controller functions
    public HeadingState getHeadingControllerState() {
        return _headingController.getHeadingState();
    }

    public void setHeadingControllerState(HeadingState state) {
        _headingController.setState(state);
    }

    public double getRotationCorrection() {
        return _headingController.getRotationCorrection(getHeading());
    }

    public void temporaryDisabledHeadingController() {
        _headingController.temporaryDisable();
    }

    public void disableHeadingController() {
        _headingController.disable();
    }

    public void initializeHeadingController() {
        _headingController.setMaintainHeading(getHeading());
    }

    public void incrementHeadingControllerAngle() {
        Rotation2d heading = getHeading();
        _headingController.setMaintainHeading(
                Rotation2d.fromDegrees(heading.getDegrees() + Constants.DriveTrain.BUMP_DEGREES));
    }

    public void decrementHeadingControllerAngle() {
        Rotation2d heading = getHeading();
        _headingController.setMaintainHeading(
                Rotation2d.fromDegrees(heading.getDegrees() - Constants.DriveTrain.BUMP_DEGREES));
    }

    public void setSnapHeading(Rotation2d heading) {
        _headingController.setSnapHeading(heading);
    }

    public void setMaintainHeading(Rotation2d heading) {
        _headingController.setMaintainHeading(heading);
    }

    public void plotTrajectory(Trajectory t, String name) {
        _field.getObject(name).setTrajectory(t);
    }
    // public void startModules() {
    // for (SwerveModule SwerveModule : _modules) {
    // SwerveModule.start();
    // }
    // }

    public void readModules() {
        for (int module = 0; module < _modules.length; module++) {
            _systemIO.measuredStates[module] = _modules[module].getState();
            _systemIO.measuredPositions[module] = _modules[module].getPosition(true);
        }
    }

    public void resetModuleEncoders() {
        for (var module : _modules) {
            module.resetEncoders();
        }
    }

    public void setModuleStates(SwerveModuleState[] states) {
        for (int module = 0; module < _modules.length; module++) {
            _modules[module].setIdealState(states[module]);
        }
    }

    public void shiftUpModules() {
        _shift.set(Value.kForward);
        // _pneumaticHub.setSolenoids(0, 1);
        // _pneumaticHub.setSolenoids(15, 0);
        _isLowGear = false;
        setKinematicLimits(HIGH_KINEMATIC_LIMITS);
        for (int i = 0; i < _modules.length; i++) {
            _modules[i].setLowGear(false);
        }
    }

    public void shiftDownModules() {
        _shift.set(Value.kReverse);
        // _pneumaticHub.setSolenoids(0, 0);
        // _pneumaticHub.setSolenoids(15, 1);
        setKinematicLimits(LOW_KINEMATIC_LIMITS);
        for (int i = 0; i < _modules.length; i++) {
            _modules[i].setLowGear(true);
        }
        _isLowGear = true;
    }

    public boolean isLowGear() {
        return _isLowGear;
    }

    public void setSetpointFromMeasuredModules() {
        System.arraycopy(
                _systemIO.measuredStates, 0, _systemIO.setpoint.moduleStates, 0, _modules.length);
        _systemIO.setpoint.chassisSpeeds = _kinematics.toChassisSpeeds(_systemIO.setpoint.moduleStates);
    }

    public void orientModules(Rotation2d moduleAngle) {
        for (int module = 0; module < _modules.length; module++) {
            _systemIO.setpoint.moduleStates[module] = new SwerveModuleState(0.0, moduleAngle);
        }
    }

    public void setControlState(ControlState state) {
        _controlState = state;
    }

    // Swerve setpoint generator functions
    public ControlState getControlState() {
        return _controlState;
    }

    public SwerveSetpoint getSetpoint() {
        return _systemIO.setpoint;
    }

    public void setKinematicLimits(KinematicLimits limits) {
        if (limits != _kinematicLimits) {
            _kinematicLimits = limits;
        }
    }

    public void updateDesiredStates() {
        Pose2d robotPoseVel = new Pose2d(
                _systemIO.desiredChassisSpeeds.vxMetersPerSecond * Constants.UPDATE_PERIOD,
                _systemIO.desiredChassisSpeeds.vyMetersPerSecond * Constants.UPDATE_PERIOD,
                Rotation2d.fromRadians(
                        _systemIO.desiredChassisSpeeds.omegaRadiansPerSecond * Constants.UPDATE_PERIOD));

        Twist2d twistVel = new Pose2d().log(robotPoseVel);
        ChassisSpeeds updatedChassisSpeeds = new ChassisSpeeds(
                twistVel.dx / Constants.UPDATE_PERIOD,
                twistVel.dy / Constants.UPDATE_PERIOD,
                twistVel.dtheta / Constants.UPDATE_PERIOD);
        _systemIO.setpoint = _swerveSetpointGenerator.generateSetpoint(
                _kinematicLimits,
                _systemIO.setpoint,
                // _systemIO.desiredChassisSpeeds,
                updatedChassisSpeeds,
                Constants.UPDATE_PERIOD);
    }

    // CTRE's driving code
    public void driveFieldCentric(ChassisSpeeds speeds) {
        var roboCentric = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getHeading());
        var swerveStates = _kinematics.toSwerveModuleStates(roboCentric);
        for (int i = 0; i < _modules.length; ++i) {
            _modules[i].setIdealState(swerveStates[i]);
        }
    }

    public void setVelocity(ChassisSpeeds chassisSpeeds) {
        _systemIO.desiredChassisSpeeds = chassisSpeeds;
    }

    public void setVelocityPose(Pose2d pose, boolean isShooter) {
        ChassisSpeeds speeds = _poseController.calculate(
                _systemIO.estimatedPose, pose, 0.0, _systemIO.heading);
        _headingController.setMaintainHeading(isShooter ? new Rotation2d(Math.PI) : new Rotation2d());
        speeds.omegaRadiansPerSecond = _headingController.getRotationCorrection(getHeading());
        _systemIO.desiredChassisSpeeds = speeds;
    }

    // public void followTrajectory(PathPlannerTrajectory.PathPlannerState desiredState) {
    //     ChassisSpeeds speeds = _trajectoryController.calculate(getEstimatedPose(), desiredState);
    //     // speeds.omegaRadiansPerSecond = 0.0;
    //     _systemIO.desiredChassisSpeeds = speeds;
    // }

    public double getYaw() {
        return _systemIO.heading.getRadians();
    }

    public double getPitch() {
        return _systemIO.pitch;
    }

    // yaw is negative to follow wpi coordinate system.
    public Rotation2d getHeading() {
        return _systemIO.heading;
    }

    public void zeroGyroscope() {
        _yawOffset = _imu.getYaw().getValue();
        readIMU();
        resetRobotPose(_systemIO.estimatedPose);
    }

    public void setGyroscopeAngle(Rotation2d rotation) {
        _yawOffset = _imu.getYaw().getValue() + rotation.getDegrees();
        readIMU();
    }

    public void readIMU() {
        _systemIO.heading = Rotation2d.fromDegrees((_imu.getYaw().getValue() - _yawOffset));
        _systemIO.pitch = Units.degreesToRadians(_imu.getPitch().getValue());
    }

    public TrajectoryConfig getConfig() {
        return new TrajectoryConfig(TRAJECTORY_FOLLOWING.maxDriveVelocity, TRAJECTORY_FOLLOWING.maxDriveAcceleration)
                .setKinematics(_kinematics)
                .addConstraint(getKinematicConstraint());
    }

    public SwerveDriveKinematicsConstraint getKinematicConstraint() {
        return new SwerveDriveKinematicsConstraint(_kinematics, TRAJECTORY_FOLLOWING.maxDriveAcceleration);
    }

    public SwerveDriveKinematics getKinematics() {
        return _kinematics;
    }

    public void updateOdometry() {
        _odometry.update(
                isRedAlliance() ? getHeading().minus(new Rotation2d(Math.PI)) : getHeading(),
                new SwerveModulePosition[] {
                        _modules[NORTH_WEST_IDX].getModulePosition(),
                        _modules[SOUTH_WEST_IDX].getModulePosition(),
                        _modules[SOUTH_EAST_IDX].getModulePosition(),
                        _modules[NORTH_EAST_IDX].getModulePosition()
                });
    }

    public Pose2d getOdometryPose() {
        return _odometry.getPoseMeters();
    }

    /**
     * Reset position and gyroOffset of odometry
     *
     * @param position is a Pose2d (Translation2d, Rotation2d)
     *                 <p>
     *                 Translation2d resets odometry (X,Y) coordinates
     *                 <p>
     *                 Rotation2d - gyroAngle = gyroOffset
     *                 <p>
     *                 If Rotation2d <> gyroAngle, then robot heading will no longer
     *                 equal IMU heading.
     */
    public void resetRobotPose(Pose2d position) {
        for (int module = 0; module < _modules.length; module++) {
            _modules[module].resetEncoders();
        }
        // setGyroscopeAngle(position.getRotation());
        error(" Rotation: " + getHeading().toString());

        // Rotation2d _rotation = _isRedAlliance ? getHeading().minus(new
        // Rotation2d(Math.PI)) : getHeading();
        // Pose2d _reset = new Pose2d(_translation, _rotation);
        _poseEstimator.resetPosition(
                getHeading(),
                new SwerveModulePosition[] {
                        _modules[NORTH_WEST_IDX].getModulePosition(),
                        _modules[SOUTH_WEST_IDX].getModulePosition(),
                        _modules[SOUTH_EAST_IDX].getModulePosition(),
                        _modules[NORTH_EAST_IDX].getModulePosition()
                },
                position);
        error("Reset robot position: " + position.toString());
        _systemIO.estimatedPose = _poseEstimator.getEstimatedPosition();
    }

    public void wantsToResetPose(Pose2d pose) {
        _wantedRestPose = pose;
        _wantsToSetPose = true;
    }

    public void moduleMetrics() {
        for (var module : _modules) {
            module.updateDashboard();
        }
    }

    public double getDistanceToGoal() {
        return _systemIO.estimatedPose
                .getTranslation()
                .getDistance(_hoverGoal.getTranslation());
    }

    public boolean isTopSpeed() {
        return Math.abs(_modules[0].getWheelVelocity()) >= (Constants.DriveTrain.MAX_MPS - 0.2);
    }

    @Override
    public void updateDashboard() {
        metric("Swerve State", _controlState.name());
        metric("Robot goal position", _hoverGoal.toString());
        metric("Current Heading", getHeading().getRadians());
        metric("Heading Controller Target", _headingController.getTargetHeading().getRadians());
        metric("Heading State", _headingController.getHeadingState().name());
        metric("Rotation State", getYaw());
        metric("Pitch Angle", getPitch());
        metric("Estimated X", _systemIO.estimatedPose.getX());
        metric("Estimated Y", _systemIO.estimatedPose.getY());
        metric("NW Angle", _modules[0].getCanCoderAngle().getRadians());
        metric("SW Angle", _modules[1].getCanCoderAngle().getRadians());
        metric("SE Angle", _modules[2].getCanCoderAngle().getRadians());
        metric("NE Angle", _modules[3].getCanCoderAngle().getRadians());
        metric("NW AngleRot", _modules[0].getCanCoderAngle().getRotations());
        metric("SW AngleRot", _modules[1].getCanCoderAngle().getRotations());
        metric("SE AngleRot", _modules[2].getCanCoderAngle().getRotations());
        metric("NE AngleRot", _modules[3].getCanCoderAngle().getRotations());
        metric("NW Angle Wanted", _systemIO.setpoint.moduleStates[0].angle.getRadians());
        metric("SW Angle Wanted", _systemIO.setpoint.moduleStates[1].angle.getRadians());
        metric("SE Angle Wanted", _systemIO.setpoint.moduleStates[2].angle.getRadians());
        metric("NE Angle Wanted", _systemIO.setpoint.moduleStates[3].angle.getRadians());
        metric("NW Velocity", _modules[0].getWheelVelocity());
        metric("SW Velocity", _modules[1].getWheelVelocity());
        metric("SE Velocity", _modules[2].getWheelVelocity());
        metric("NE Velocity", _modules[3].getWheelVelocity());
        metric("NW Velocity Wanted", _modules[0].getWantedSpeed());
        metric("SW Velocity Wanted", _modules[1].getWantedSpeed());
        metric("SE Velocity Wanted", _modules[2].getWantedSpeed());
        metric("NE Velocity Wanted", _modules[3].getWantedSpeed());
        metric("NW Drive Velocity", _modules[0].getDriveRPM() / 60);
        metric("SW Drive Velocity", _modules[1].getDriveRPM() / 60);
        metric("NE Drive Velocity", _modules[2].getDriveRPM() / 60);
        metric("SE Drive Velocity", _modules[3].getDriveRPM() / 60);
        metric("NW State Velocity", _modules[0].getStateMPS());
        metric("SW State Velocity", _modules[1].getStateMPS());
        metric("NE State Velocity", _modules[2].getStateMPS());
        metric("SE State Velocity", _modules[3].getStateMPS());
        metric("NW Current", _modules[0].getDriveMotorCurrent());
        metric("SW Current", _modules[1].getDriveMotorCurrent());
        metric("SE Current", _modules[2].getDriveMotorCurrent());
        metric("NE Current", _modules[3].getDriveMotorCurrent());
        metric(
                "Distance to goal node",
                _systemIO.estimatedPose
                        .getTranslation()
                        .getDistance(_hoverGoal.getTranslation()));
        metric("Drivetrain Speed", getSpeed());
        metric("Is Low Gear", isLowGear());
        SmartDashboard.putData(_field);
        moduleMetrics();
    }

    public enum ControlState {
        NEUTRAL(0),
        MANUAL(1),
        POSITION(2),
        ROTATION(3),
        TRAJECTORY(4);

        private final int _value;

        ControlState(int value) {
            _value = value;
        }

        public int getValue() {
            return _value;
        }
    }

    public enum Mode {
        NORMAL(0),
        SLOW(1),
        VISION(2);

        private final int _value;

        Mode(int value) {
            _value = value;
        }

        public int getValue() {
            return _value;
        }
    }

    public void setMode(Mode mode) {
        _mode = mode;
    }

    public Mode getMode() {
        return _mode;
    }

    public boolean isRedAlliance() {
        return true;
    }

    public Pose2d getHoverGoal() {
        return _hoverGoal;
    }

    public void setHoverGoal(Pose2d pose) {
        _hoverGoal = pose;
    }

    public ChassisSpeeds getMeasuredChassisSpeeds() {
        return _kinematics.toChassisSpeeds(_systemIO.measuredStates);
    }

    public ChassisSpeeds getDesiredChassisSpeeds() {
        return _systemIO.desiredChassisSpeeds;
    }

    public double getDesiredSpeed() {
        return Math.hypot(getDesiredChassisSpeeds().vxMetersPerSecond, getDesiredChassisSpeeds().vyMetersPerSecond);
    }

    public void setShiftLockout(boolean lock) {
        _shiftLockout = lock;
    }

    public double getSpeed() {
        return Math.hypot(getMeasuredChassisSpeeds().vxMetersPerSecond, getMeasuredChassisSpeeds().vyMetersPerSecond);
    }

    public void autoShifter() {
        double speed = getSpeed();

        double LockoutTime = 500;
        if (speed > Constants.DriveTrain.SHIFT_UP_SPEED_MPS && getDesiredSpeed() > SHIFT_UP_SPEED_MPS) {
            if (!_shiftLockout) {
                _shiftLockout = true;
                error("Shifting up");
                _shiftTime = System.currentTimeMillis();
                shiftUpModules();
            }
            if (_shiftTime + LockoutTime < System.currentTimeMillis()) {
                _shiftLockout = false;
            }
        }

        if (speed < Constants.DriveTrain.SHIFT_DOWN_SPEED_MPS) {
            if (!_shiftLockout) {
                _shiftTime = System.currentTimeMillis();
                _shiftLockout = true;
                shiftDownModules();
            }
            if (_shiftTime + LockoutTime < System.currentTimeMillis()) {
                _shiftLockout = false;
            }
        }

    }

    /* Vision Stuff */
    @Override
    public void dataPeriodic(double timestamp) {

        // current assumption is that because this is in a different thread a data race
        // is occurring where a reset doesn't occur until after a measurement(or during)
        // which is causing the estimated pose to be wrong.
        if (_wantsToSetPose) {
            resetRobotPose(_wantedRestPose);
            _wantsToSetPose = false;
        } else {
            _poseEstimator.update(getHeading(), _systemIO.measuredPositions);
            Pose2d prevEstimatedPose = _poseEstimator.getEstimatedPosition();
            List<EstimatedRobotPose> cameraPoses = Stream.of(
                    _photonProcessor.getNorthCameraEstimatedGlobalPose(prevEstimatedPose),
                    _photonProcessor.getSouthCameraEstimatedGlobalPose(prevEstimatedPose),
                    _photonProcessor.getWestCameraEstimatedGlobalPose(prevEstimatedPose),
                    _photonProcessor.getEastCameraEstimatedGlobalPose(prevEstimatedPose))
                    .flatMap(Optional::stream)
                    .filter(cameraPose -> isValidMeasurement(cameraPose.estimatedPose))
                    .collect(Collectors.toList());

            cameraPoses.forEach(cameraPose -> {
                dynamicallyChangeDeviations(cameraPose.estimatedPose, prevEstimatedPose);
                _poseEstimator.addVisionMeasurement(cameraPose.estimatedPose.toPose2d(), cameraPose.timestampSeconds);
            });

            _systemIO.estimatedPose = _poseEstimator.getEstimatedPosition();
        }
        _field.setRobotPose(_systemIO.estimatedPose);
    }

    public Pose2d getEstimatedPose() {
        return _systemIO.estimatedPose;
    }

    public boolean isValidMeasurement(Pose3d measurement) {
        final double heightThreshold = Units.inchesToMeters(30);
        final double fieldBuffer = Units.inchesToMeters(6); // add a 6 inch buffer to the field boundaries

        boolean isTargetWithinHeight = measurement.getZ() < (heightThreshold + heightThreshold * 0.1); // allow for a
                                                                                                       // 10% error in
                                                                                                       // height
                                                                                                       // measurement
        boolean isMeasurementInField = (measurement.getX() >= -fieldBuffer
                && measurement.getX() <= FieldConstants.fieldLength + fieldBuffer)
                && (measurement.getY() >= -fieldBuffer
                        && measurement.getY() <= FieldConstants.fieldWidth + fieldBuffer);
        return isTargetWithinHeight && isMeasurementInField;
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
