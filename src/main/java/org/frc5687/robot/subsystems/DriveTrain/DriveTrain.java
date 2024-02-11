/* Team 5687 (C)2020-2022 */
package org.frc5687.robot.subsystems.DriveTrain;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static org.frc5687.robot.Constants.DriveTrain.*;

import org.frc5687.lib.swerve.SwerveSetpoint;
import org.frc5687.lib.swerve.SwerveSetpointGenerator;
import org.frc5687.lib.swerve.SwerveSetpointGenerator.KinematicLimits;
import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotMap;
import org.frc5687.robot.subsystems.OutliersSubsystem;
import org.frc5687.robot.subsystems.SwerveModule;
import org.frc5687.robot.util.*;

public class DriveTrain extends OutliersSubsystem {
    public enum ControlState {
        /* Neutral state */
        NEUTRAL(0),
        /* Driving With Joystick */
        MANUAL(1),

        /* Use pose controller to drive */
        POSITION(2),

        /* Rotation only control */
        ROTATION(3),
        /* Trajectory driving control */
        TRAJECTORY(4);

        private final int _value;

        ControlState(int value) {
            _value = value;
        }

        public int getValue() {
            return _value;
        }
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
        Pose2d odometryPose = new Pose2d();

        SwerveSetpoint setpoint = new SwerveSetpoint(new ChassisSpeeds(), new SwerveModuleState[4]);
    }

    // Order we define swerve modules in kinematics
    private final SwerveModule[] _modules;
    private static final int NORTH_WEST_IDX = 0;
    private static final int SOUTH_WEST_IDX = 1;
    private static final int SOUTH_EAST_IDX = 2;
    private static final int NORTH_EAST_IDX = 3;

    private final SwerveSetpointGenerator _swerveSetpointGenerator;
    private KinematicLimits _kinematicLimits = LOW_KINEMATIC_LIMITS;

    private final DoubleSolenoid _shift;
    private final Compressor _compressor;

    private final BaseStatusSignal[] _moduleSignals;

    private final SwerveDriveKinematics _kinematics;
    private final SwerveDriveOdometry _odometry;

    // controllers [Heading, Pose, Trajectory]
    private ControlState _controlState;

    // IMU (Pigeon)
    private final Pigeon2 _imu;
    private double _yawOffset;

    private final Field2d _field;
    private Mode _mode = Mode.NORMAL;
    private Pose2d _hoverGoal;

    private boolean _shiftLockout = false;
    private long _shiftTime = 0;

    private boolean _hasShiftInit = false;
    private boolean _isLowGear;

    private final SystemIO _systemIO;
    private HeadingDriveController _headingDriveController;
    private AutoAlignDriveController _alignDriveController;

    private boolean _useHeadingController;

    public DriveTrain(
            OutliersContainer container,
            Pigeon2 imu) {
        super(container);

        _shift = new DoubleSolenoid(
                PneumaticsModuleType.REVPH,
                RobotMap.PCM.SHIFTER_HIGH,
                RobotMap.PCM.SHIFTER_LOW);

        _compressor = new Compressor(PneumaticsModuleType.REVPH);
        _compressor.enableAnalog(
                Constants.DriveTrain.MIN_PSI,
                Constants.DriveTrain.MAX_PSI);

        // configure our system IO and pigeon;
        _imu = imu;
        _systemIO = new SystemIO();

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

        // frequency in Hz
        configureSignalFrequency(250);

        // configure startup offset.
        _yawOffset = _imu.getYaw().getValue();
        readIMU();

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


        _swerveSetpointGenerator = new SwerveSetpointGenerator(
                _kinematics,
                new Translation2d[] {
                        _modules[NORTH_WEST_IDX].getModuleLocation(),
                        _modules[SOUTH_WEST_IDX].getModuleLocation(),
                        _modules[SOUTH_EAST_IDX].getModuleLocation(),
                        _modules[NORTH_EAST_IDX].getModuleLocation()
                });

        _headingDriveController = new HeadingDriveController();
        _alignDriveController = new AutoAlignDriveController();

        _useHeadingController = false;

        _field = new Field2d();
        _hoverGoal = new Pose2d();
        _controlState = ControlState.MANUAL;
        _isLowGear = true;

        zeroGyroscope();

        readModules();
        setSetpointFromMeasuredModules();

        // logMetrics("SE Current", "NE Current", "NW Current", "SW Current");
    }


    protected void configureSignalFrequency(double frequency) {
        for (var signal : _moduleSignals) {
            signal.setUpdateFrequency(frequency);
        }
        _imu.getYaw().setUpdateFrequency(frequency);
        _imu.getPitch().setUpdateFrequency(frequency);
        _imu.getRoll().setUpdateFrequency(frequency);
        _imu.getAngularVelocityZDevice().setUpdateFrequency(frequency);
    }

    public void readSignals() {
        BaseStatusSignal.waitForAll(0.1, _moduleSignals);
        readIMU();
        readModules();
    }
    
    public void setEnableHeadingController(boolean enabled) {
        _useHeadingController = enabled;
    }

    public void setHeadingControllerGoal(Rotation2d goal) {
        _headingDriveController.setTargetHeading(goal);
    }

    public void setHeadingMaintainGoal(Rotation2d goal) {
        _headingDriveController.maintainCurrentHeading(goal);
    }

    private void updateHeadingController() {
        synchronized (_systemIO) {
            _systemIO.desiredChassisSpeeds = _headingDriveController.update(
                _systemIO.desiredChassisSpeeds,
                getHeading()
            );
        }
    }

    private void updateAutoAlignController() {
        synchronized (_systemIO) {
            _systemIO.desiredChassisSpeeds = _alignDriveController.updateAutoAlign(_hoverGoal);
        }
    }

    public boolean isAutoAlignComplete() {
        return _alignDriveController.isAutoAlignComplete();
    }

    @Override
    public void periodic() {
        super.periodic();
        if (!_hasShiftInit) {
            shiftDownModules();
            _hasShiftInit = true;
        }

        if (!_useHeadingController) {
            _headingDriveController.reset(getHeading());
        }

        readSignals();

        /* Update odometry */
        _odometry.update(getHeading(), _systemIO.measuredPositions);

        switch (_controlState) {
            case NEUTRAL:
                break;  
            case MANUAL:
                updateHeadingController();
                break;
            case POSITION: 
                updateAutoAlignController();
                break;
            case ROTATION: 
                break;
            case TRAJECTORY:
                break;

        }
        updateDesiredStates();
        setModuleStates(_systemIO.setpoint.moduleStates);
    }

    public void updateDesiredStates() {
        // This is to avoid skew when driving and rotating.
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
                updatedChassisSpeeds,
                Constants.UPDATE_PERIOD);
    }

    /* Module Control Start */
    public void readModules() {
        for (int module = 0; module < _modules.length; module++) {
            _systemIO.measuredStates[module] = _modules[module].getState();
            _systemIO.measuredPositions[module] = _modules[module].getPosition();
        }
    }

    public void resetModuleEncoders() {
        for (var module : _modules) {
            module.resetEncoders();
        }
    }

    public void setModuleStates(SwerveModuleState[] states) {
        for (int module = 0; module < _modules.length; module++) {
            _modules[module].setModuleState(states[module]);
        }
    }

    public SwerveModulePosition[] getSwerveModuleMeasuredPositions() {
        return _systemIO.measuredPositions;
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
    /* Module Control End */

    public void plotTrajectory(Trajectory t, String name) {
        _field.getObject(name).setTrajectory(t);
    }

    public void setControlState(ControlState state) {
        _controlState = state;
    }

    public ControlState getControlState() {
        return _controlState;
    }

    /* Swerve Control Start */
    public void setVelocity(ChassisSpeeds chassisSpeeds) {
        _systemIO.desiredChassisSpeeds = chassisSpeeds;
    }

    public SwerveSetpoint getSetpoint() {
        return _systemIO.setpoint;
    }

    public TrajectoryConfig getConfig() {
        return new TrajectoryConfig(TRAJECTORY_FOLLOWING.maxDriveVelocity, TRAJECTORY_FOLLOWING.maxDriveAcceleration)
                .setKinematics(_kinematics)
                .addConstraint(getKinematicConstraint());
    }

    /* Kinematics Stuff Start */
    public SwerveDriveKinematicsConstraint getKinematicConstraint() {
        return new SwerveDriveKinematicsConstraint(_kinematics, TRAJECTORY_FOLLOWING.maxDriveAcceleration);
    }

    public void setKinematicLimits(KinematicLimits limits) {
        if (limits != _kinematicLimits) {
            _kinematicLimits = limits;
        }
    }

    public KinematicLimits getKinematicLimits() {
        return _kinematicLimits;
    }

    public SwerveDriveKinematics getKinematics() {
        return _kinematics;
    }
    /* Odometry And Pose Estimator Start */
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

    public boolean isTopSpeed() {
        return Math.abs(_modules[0].getWheelVelocity()) >= (Constants.DriveTrain.MAX_MPS - 0.2);
    }

    @Override
    public void updateDashboard() {
        metric("Swerve State", _controlState.name());
        metric("Current Heading", getHeading().getRadians());

        // metric("Robot goal position", _hoverGoal.toString());
        // metric("Heading Controller Target", _headingController.getTargetHeading().getRadians());
        // metric("Heading State", _headingController.getHeadingState().name());
        // metric("Rotation State", getYaw());
        // // metric("Pitch Angle", getPitch());
        // metric("Estimated X", _systemIO.estimatedPose.getX());
        // metric("Estimated Y", _systemIO.estimatedPose.getY());
        // metric("NW Angle", _modules[0].getCanCoderAngle().getRadians());
        // metric("SW Angle", _modules[1].getCanCoderAngle().getRadians());
        // metric("SE Angle", _modules[2].getCanCoderAngle().getRadians());
        // metric("NE Angle", _modules[3].getCanCoderAngle().getRadians());
        // // metric("NW AngleRot", _modules[0].getCanCoderAngle().getRotations());
        // // metric("SW AngleRot", _modules[1].getCanCoderAngle().getRotations());
        // // metric("SE AngleRot", _modules[2].getCanCoderAngle().getRotations());
        // // metric("NE AngleRot", _modules[3].getCanCoderAngle().getRotations());
        // metric("NW Angle Wanted", _systemIO.setpoint.moduleStates[0].angle.getRadians());
        // metric("SW Angle Wanted", _systemIO.setpoint.moduleStates[1].angle.getRadians());
        // metric("SE Angle Wanted", _systemIO.setpoint.moduleStates[2].angle.getRadians());
        // metric("NE Angle Wanted", _systemIO.setpoint.moduleStates[3].angle.getRadians());
        // // metric("NW Velocity", _modules[0].getWheelVelocity());
        // // metric("SW Velocity", _modules[1].getWheelVelocity());
        // // metric("SE Velocity", _modules[2].getWheelVelocity());
        // // metric("NE Velocity", _modules[3].getWheelVelocity());
        // // metric("NW Velocity Wanted", _modules[0].getWantedSpeed());
        // // metric("SW Velocity Wanted", _modules[1].getWantedSpeed());
        // // metric("SE Velocity Wanted", _modules[2].getWantedSpeed());
        // // metric("NE Velocity Wanted", _modules[3].getWantedSpeed());
        // // metric("NW Drive Velocity", _modules[0].getDriveRPM() / 60);
        // // metric("SW Drive Velocity", _modules[1].getDriveRPM() / 60);
        // // metric("NE Drive Velocity", _modules[2].getDriveRPM() / 60);
        // // metric("SE Drive Velocity", _modules[3].getDriveRPM() / 60);
        // // metric("NW State Velocity", _modules[0].getStateMPS());
        // // metric("SW State Velocity", _modules[1].getStateMPS());
        // // metric("NE State Velocity", _modules[2].getStateMPS());
        // // metric("SE State Velocity", _modules[3].getStateMPS());
        // metric("NW Current", _modules[0].getDriveMotorCurrent());
        // metric("SW Current", _modules[1].getDriveMotorCurrent());
        // metric("SE Current", _modules[2].getDriveMotorCurrent());
        // metric("NE Current", _modules[3].getDriveMotorCurrent());
        // // metric("NW Camera Estimate Pose", _photonProcessor.getNorthWestCameraEstimatedGlobalPose(_systemIO.estimatedPose).toString());
        // // metric("SW Camera Estimate Pose", _photonProcessor.getSouthWestCameraEstimatedGlobalPose(_systemIO.estimatedPose).toString());
        // // metric("SE Camera Estimate Pose", _photonProcessor.getSouthEastCameraEstimatedGlobalPose(_systemIO.estimatedPose).toString());
        // // metric("NE Camera Estimate Pose", _photonProcessor.getNorthEastCameraEstimatedGlobalPose(_systemIO.estimatedPose).toString());
        // metric("Distance to goal node",
        //         _systemIO.estimatedPose
        //                 .getTranslation()
        //                 .getDistance(_hoverGoal.getTranslation()));
        // metric("Drivetrain Speed", getSpeed());
        // metric("Is Low Gear", isLowGear());
        // metric("Tank Pressure PSI", _compressor.getPressure());
        SmartDashboard.putData(_field);
        // moduleMetrics();
    }

    public void moduleMetrics() {
        for (var module : _modules) {
            module.updateDashboard();
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
        return true; // TODO: We need to fix this. We are not always red :)
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

    public double getSpeed() {
        return Math.hypot(getMeasuredChassisSpeeds().vxMetersPerSecond, getMeasuredChassisSpeeds().vyMetersPerSecond);
    }

    /* Shift stuff start */
    public void shiftUpModules() {
        _shift.set(Value.kForward);
        _isLowGear = false;
        setKinematicLimits(HIGH_KINEMATIC_LIMITS);
        for (int i = 0; i < _modules.length; i++) {
            _modules[i].setLowGear(false);
        }
    }

    public void shiftDownModules() {
        _shift.set(Value.kReverse);
        setKinematicLimits(LOW_KINEMATIC_LIMITS);
        for (int i = 0; i < _modules.length; i++) {
            _modules[i].setLowGear(true);
        }
        _isLowGear = true;
    }

    public boolean isLowGear() {
        return _isLowGear;
    }

    public void setShiftLockout(boolean lock) {
        _shiftLockout = lock;
    }

    public void autoShifter() {
        double speed = getSpeed();
        if (speed > Constants.DriveTrain.SHIFT_UP_SPEED_MPS && getDesiredSpeed() > SHIFT_UP_SPEED_MPS) {
            if (!_shiftLockout) {
                _shiftLockout = true;
                error("Shifting up");
                _shiftTime = System.currentTimeMillis();
                shiftUpModules();
            }
            if (_shiftTime + Constants.DriveTrain.SHIFT_LOCKOUT < System.currentTimeMillis()) {
                _shiftLockout = false;
            }
        }

        if (speed < Constants.DriveTrain.SHIFT_DOWN_SPEED_MPS) {
            if (!_shiftLockout) {
                _shiftTime = System.currentTimeMillis();
                _shiftLockout = true;
                shiftDownModules();
            }
            if (_shiftTime + Constants.DriveTrain.SHIFT_LOCKOUT < System.currentTimeMillis()) {
                _shiftLockout = false;
            }
        }
    }

    public double getYaw() {
        return _systemIO.heading.getRadians();
    }

    public double getPitch() {
        return _systemIO.pitch;
    }

    public Rotation2d getHeading() {
        return _systemIO.heading;
    }

    public void zeroGyroscope() {
        _yawOffset = _imu.getYaw().getValue();
        readIMU();
    }

    public void setGyroscopeAngle(Rotation2d rotation) {
        _yawOffset = _imu.getYaw().getValue() + rotation.getDegrees();
        readIMU();
    }

    public void readIMU() {
        double yawDegrees = BaseStatusSignal.getLatencyCompensatedValue(_imu.getYaw(),
                _imu.getAngularVelocityZDevice());
        _systemIO.heading = Rotation2d.fromDegrees(yawDegrees - _yawOffset);
        _systemIO.pitch = Units.degreesToRadians(_imu.getPitch().getValue());
    }
}
