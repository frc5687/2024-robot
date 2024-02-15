/* Team 5687 (C)2020-2022 */
package org.frc5687.robot.subsystems.DriveTrain;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import static org.frc5687.robot.Constants.DriveTrain.*;

import java.util.Optional;

import org.frc5687.lib.control.SwerveHeadingController;
import org.frc5687.lib.control.SwerveHeadingController.HeadingState;
import org.frc5687.lib.swerve.SwerveSetpoint;
import org.frc5687.lib.swerve.SwerveSetpointGenerator;
import org.frc5687.lib.swerve.SwerveSetpointGenerator.KinematicLimits;
import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotMap;
import org.frc5687.robot.RobotState;
import org.frc5687.robot.subsystems.OutliersSubsystem;
import org.frc5687.robot.subsystems.SwerveModule;
import org.frc5687.robot.Constants.FieldConstants;
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
    private boolean _compressInit;

    private final BaseStatusSignal[] _moduleSignals;

    private final SwerveDriveKinematics _kinematics;
    private final SwerveDriveOdometry _odometry;

    private final SwerveHeadingController _headingController;

    // controllers [Heading, Pose, Trajectory]
    private ControlState _controlState;

    // IMU (Pigeon)
    private final Pigeon2 _imu;
    private double _yawOffset;

    private Pose2d _hoverGoal;

    private boolean _shiftLockout = false;
    private long _shiftTime = 0;

    private boolean _hasShiftInit = false;
    private boolean _isLowGear;

    private final SystemIO _systemIO;
    // private YawDriveController _yawDriveController;
    private AutoPoseDriveController _poseDriveController;
    private RobotState _robotState = RobotState.getInstance();

    private boolean _useHeadingController;

    public DriveTrain(
            OutliersContainer container,
            Pigeon2 imu) {
        super(container);

        _shift = new DoubleSolenoid(
                PneumaticsModuleType.REVPH,
                RobotMap.PCM.SHIFTER_HIGH,
                RobotMap.PCM.SHIFTER_LOW);
        // create compressor, compressor logic
        _compressor = new Compressor(PneumaticsModuleType.REVPH);
        _compressInit = false;

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
                        _modules[NORTH_WEST_IDX].getPosition(),
                        _modules[SOUTH_WEST_IDX].getPosition(),
                        _modules[SOUTH_EAST_IDX].getPosition(),
                        _modules[NORTH_EAST_IDX].getPosition()
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

        _headingController = new SwerveHeadingController(Constants.UPDATE_PERIOD);

        _poseDriveController = new AutoPoseDriveController();

        // this `false` value doesn't mean that the heading controller is disabled.
        // as of 02/13/24, it gets initialized to true in the Drive command
        // this default value can and will be overridden by commands - xavier bradford
        _useHeadingController = false;

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

    /* Heading Controller Start */
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
    /* Heading Controller End */

    private void updateAutoPoseController() {
        _systemIO.desiredChassisSpeeds = _poseDriveController.updateAutoAlign(_robotState.getEstimatedPose());
    }

    public boolean isAutoPoseComplete() {
        return _poseDriveController.isAutoAlignComplete();
    }

    @Override
    public void periodic() {
        super.periodic();
        if (!_hasShiftInit) {
            shiftDownModules();
            _hasShiftInit = true;
        }

        if(!_compressInit){
            _compressor.enableAnalog(Constants.DriveTrain.MAX_PSI, Constants.DriveTrain.MAX_PSI + 3);
            if (_compressor.getPressureSwitchValue() && !_compressInit){
                _compressor.enableAnalog(Constants.DriveTrain.MIN_PSI, Constants.DriveTrain.MAX_PSI);
                _compressInit = true;
            }
        }

        readSignals();

        /* Update odometry */
        _odometry.update(getHeading(), _systemIO.measuredPositions);

        switch (_controlState) {
            case NEUTRAL:
                break;
            case MANUAL:
                break;
            case POSITION: 
                updateAutoPoseController();
                break;
            case ROTATION:
                break;
            case TRAJECTORY:
                break;

        }
        updateDesiredStates();
        setModuleStates(_systemIO.setpoint.moduleStates);
    }

    public void setControlState(ControlState state) {
        _controlState = state;
    }

    public ControlState getControlState() {
        return _controlState;
    }

    public void setVelocity(ChassisSpeeds chassisSpeeds) {
        _systemIO.desiredChassisSpeeds = chassisSpeeds;
    }

    public SwerveSetpoint getSetpoint() {
        return _systemIO.setpoint;
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

    public SwerveDriveKinematicsConstraint getKinematicConstraint() {
        return new SwerveDriveKinematicsConstraint(_kinematics, TRAJECTORY_FOLLOWING.maxDriveAcceleration);
    }

    public TrajectoryConfig getConfig() {
        return new TrajectoryConfig(TRAJECTORY_FOLLOWING.maxDriveVelocity, TRAJECTORY_FOLLOWING.maxDriveAcceleration)
                .setKinematics(_kinematics)
                .addConstraint(getKinematicConstraint());
    }
    /* Trajectory Info End */

    /* Kinematics Stuff Start */
    public SwerveDriveKinematics getKinematics() {
        return _kinematics;
    }

    /* Kinematic limit for the Setpoint Generator */
    public void setKinematicLimits(KinematicLimits limits) {
        if (limits != _kinematicLimits) {
            _kinematicLimits = limits;
        }
    }

    public KinematicLimits getKinematicLimits() {
        return _kinematicLimits;
    }
    /* Kinematics End */

    /* Odometry And Pose Estimator Start */
    public void updateOdometry() {
        _odometry.update(
                isRedAlliance() ? getHeading().minus(new Rotation2d(Math.PI)) : getHeading(),
                new SwerveModulePosition[] {
                        _modules[NORTH_WEST_IDX].getPosition(),
                        _modules[SOUTH_WEST_IDX].getPosition(),
                        _modules[SOUTH_EAST_IDX].getPosition(),
                        _modules[NORTH_EAST_IDX].getPosition()
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
        metric("Tank Pressure PSI", _compressor.getPressure());
        // moduleMetrics();
    }

    public void moduleMetrics() {
        for (var module : _modules) {
            module.updateDashboard();
        }
    }

    public boolean isRedAlliance() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == Alliance.Red;
        }
        error("Could not check if alliance was red, driver station was not connected");
        return false;
    }

    public Pose2d getHoverGoal() {
        return _hoverGoal;
    }

    public void setHoverGoal(Pose2d pose) {
        _hoverGoal = pose;
        _poseDriveController.setTargetPoint(_hoverGoal);
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
            _modules[i].startShift();
            _modules[i].setLowGear(false);
        }
    }

    public void shiftDownModules() {
        _shift.set(Value.kReverse);
        setKinematicLimits(LOW_KINEMATIC_LIMITS);
        for (int i = 0; i < _modules.length; i++) {
            _modules[i].startShift();
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
    /* Shift stuff end */
    

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
