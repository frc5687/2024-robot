/* Team 5687  */
package org.frc5687.robot.subsystems;

import static org.frc5687.robot.Constants.SwerveModule.*;

import org.frc5687.lib.drivers.OutliersTalon;
import org.frc5687.robot.Constants;

// import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
// import com.ctre.phoenix.sensors.SensorTimeBase;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Swerve Module Code Created in the shadow of
// the death of diffy swerve by Linus Krenkel
// using VelocityFOC and PositionVoltage 
public class SwerveModule {

    private final OutliersTalon _driveMotor;
    private final OutliersTalon _steeringMotor;
    private final CANcoder _encoder;

    private boolean _isLowGear;

    private Translation2d _positionVector;

    private SwerveModuleState _goal;

    private final StatusSignal<Double> _driveVelocityRotationsPerSec;
    private final StatusSignal<Double> _drivePositionRotations;
    private final StatusSignal<Double> _steeringVelocityRotationsPerSec;
    private final StatusSignal<Double> _steeringPositionRotations;

    private final BaseStatusSignal[] _signals;

    private VelocityTorqueCurrentFOC _velocityTorqueCurrentFOC;
    // private final PositionVoltage _positionVoltage;

    private SwerveModulePosition _internalState = new SwerveModulePosition();

    private double _rotPerMet;
    private double _gearRatio;
    private double _metPerRot;

    private double _wantedSpeed;
    private double _stateMPS;

    public SwerveModule(
            SwerveModule.ModuleConfiguration config,
            int steeringMotorID,
            int driveMotorID,
            int encoderPort) {

        _driveMotor = new OutliersTalon(driveMotorID, config.canBus, "Drive");
        _steeringMotor = new OutliersTalon(steeringMotorID, config.canBus, "Steer");

        _driveMotor.configure(Constants.SwerveModule.CONFIG);
        _steeringMotor.configure(Constants.SwerveModule.STEER_CONFIG);
        _driveMotor.configureClosedLoop(Constants.SwerveModule.DRIVE_CONTROLLER_CONFIG);
        _steeringMotor.configureClosedLoop(Constants.SwerveModule.STEER_CONTROLLER_CONFIG);
        _isLowGear = false;

        _velocityTorqueCurrentFOC = new VelocityTorqueCurrentFOC(0.0, 0.0, 0, 0, true, false, false);
        // _positionVoltage = new PositionVoltage(0.0);

        _goal = new SwerveModuleState(0.0, getCanCoderAngle());

        _encoder = new CANcoder(encoderPort, config.canBus);
        CANcoderConfiguration CANfig = new CANcoderConfiguration();
        // set units of the CANCoder to radians, with velocity being radians per second
        CANfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        CANfig.MagnetSensor.MagnetOffset = config.encoderOffset;
        CANfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        _encoder.getConfigurator().apply(CANfig);

        FeedbackConfigs feedback = new FeedbackConfigs();
        feedback.FeedbackRemoteSensorID = encoderPort;
        feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        // feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        feedback.RotorToSensorRatio = Constants.SwerveModule.GEAR_RATIO_STEER;
        _steeringMotor.configureFeedback(feedback);

        _metPerRot = 2 * Math.PI * Constants.SwerveModule.WHEEL_RADIUS;
        _rotPerMet = 1 / _metPerRot;

        _positionVector = config.position;

        _drivePositionRotations = _driveMotor.getPosition();
        _driveVelocityRotationsPerSec = _driveMotor.getVelocity();
        _steeringPositionRotations = _encoder.getPosition();
        _steeringVelocityRotationsPerSec = _encoder.getVelocity();

        _driveVelocityRotationsPerSec.setUpdateFrequency(1 / kDt);
        _drivePositionRotations.setUpdateFrequency(1 / kDt);

        _steeringVelocityRotationsPerSec.setUpdateFrequency(1 / kDt);
        _steeringPositionRotations.setUpdateFrequency(1 / kDt);

        _signals = new BaseStatusSignal[4];
        _signals[0] = _driveVelocityRotationsPerSec;
        _signals[1] = _drivePositionRotations;
        _signals[2] = _steeringVelocityRotationsPerSec;
        _signals[3] = _steeringPositionRotations;
        // _controlState = ControlState.OFF;
        System.out.println("Module has been constructed!!");
    }

    public SwerveModulePosition getPosition(boolean refresh) {
        if (refresh) {
            /* Refresh all signals */
            _drivePositionRotations.refresh();
            _driveVelocityRotationsPerSec.refresh();
            _steeringPositionRotations.refresh();
            _steeringVelocityRotationsPerSec.refresh();
        }

        /* Now latency-compensate our signals */
        double drive_rot = BaseStatusSignal.getLatencyCompensatedValue(_drivePositionRotations,
                _driveVelocityRotationsPerSec);
        double angle_rot = BaseStatusSignal.getLatencyCompensatedValue(_steeringPositionRotations,
                _steeringVelocityRotationsPerSec);

        /* And push them into a SwerveModuleState object to return */
        _internalState.distanceMeters = drive_rot / (_rotPerMet * (
            _isLowGear ? Constants.SwerveModule.GEAR_RATIO_DRIVE_LOW
        : Constants.SwerveModule.GEAR_RATIO_DRIVE_HIGH));
        // _internalState.distanceMeters = getWheelDistance();
        /* Angle is already in terms of steer rotations */
        _internalState.angle = Rotation2d.fromRotations(angle_rot);

        return _internalState;
    }

    // public ControlState getControlState() {
    // return _controlState;
    // }
    // public void setControlState(ControlState state) {
    // _controlState = state;
    // }
    public BaseStatusSignal[] getSignals() {
        return _signals;
    }

    public void setIdealState(SwerveModuleState state) {
        SmartDashboard.putNumber("/inputVelocityMPS", state.speedMetersPerSecond);
        SmartDashboard.putNumber("/wantedAngleRotations", state.angle.getRotations());
        SmartDashboard.putNumber("/currentAngleRotations", getCanCoderAngle().getRotations());

        if (Math.abs(state.speedMetersPerSecond) < 0.1) {
            stopAll();
            // System.out.println("speedMPS < 0.1");
        } else {
            state = SwerveModuleState.optimize(state, getCanCoderAngle());
            _goal = state;
            setModuleState(_goal);
        }
    }

    public void setModuleState(SwerveModuleState state) {
        // SwerveModuleState optimized = SwerveModuleState.optimize(state,
        // _internalState.angle);
        _stateMPS = state.speedMetersPerSecond;
        _wantedSpeed = state.speedMetersPerSecond
                * (_isLowGear ? Constants.SwerveModule.GEAR_RATIO_DRIVE_LOW
                        : Constants.SwerveModule.GEAR_RATIO_DRIVE_HIGH)
                * _rotPerMet;
        double position = state.angle.getRotations();
        _driveMotor.setControl(_velocityTorqueCurrentFOC.withVelocity(_wantedSpeed));
        _steeringMotor.setPositionVoltage(position);
        SmartDashboard.putNumber("/actualSpeed", _driveMotor.getVelocity().getValue());
        SmartDashboard.putNumber("/wantedPosition", position);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getWheelVelocity(), getCanCoderAngle());
    }

    public double getDriveMotorVoltage() {
        return _driveMotor.getSupplyVoltage().getValue();
    }

    public double getDriveMotorCurrent(){
        return _driveMotor.getSupplyCurrent().getValue();
    }

    public double getSteeringMotorVoltage() {
        return _steeringMotor.getSupplyVoltage().getValue();
    }

    public double getSwerveModuleVoltage() {
        return (getDriveMotorVoltage() + getSteeringMotorVoltage());
    }

    public void stopAll() {
        _driveMotor.stopMotor();
        _steeringMotor.stopMotor();
    }

    public double getEncoderAngleDouble() {
        return _encoder.getAbsolutePosition().getValue();
    }

    public Rotation2d getCanCoderAngle() {
        if (_encoder == null) {
            return Rotation2d.fromDegrees(0);
        } else {
            return Rotation2d.fromRotations(_encoder.getAbsolutePosition().getValue());
        }
    }

    public void setLowGear(boolean isLowGear){
        _isLowGear = isLowGear;
        _velocityTorqueCurrentFOC = _velocityTorqueCurrentFOC.withSlot(isLowGear ? 0 : 1);
    }

    public double getDriveRPM() {
        return OutliersTalon.rotationsPerSecToRPM(_driveVelocityRotationsPerSec.getValue(), 1.0);
    }

    public double getTurningRPM() {
        return OutliersTalon.rotationsPerSecToRPM(_steeringVelocityRotationsPerSec.getValue(), 1.0);
    }

    public double getWheelVelocity() {
        return getWheelAngularVelocity() * Constants.SwerveModule.WHEEL_RADIUS; // Meters per sec.
    }

    public double getWantedSpeed(){
        return _wantedSpeed;
    }

    public double getStateMPS(){
        return _stateMPS;
    }

    public double getWheelAngularVelocity() {
        return Units.rotationsPerMinuteToRadiansPerSecond(getDriveRPM() /
                (_isLowGear ? Constants.SwerveModule.GEAR_RATIO_DRIVE_LOW
                        : Constants.SwerveModule.GEAR_RATIO_DRIVE_HIGH));
    }

    public Translation2d getModuleLocation() {
        return _positionVector;
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(getWheelDistance(), getCanCoderAngle());
    }

    public double getWheelDistance() {
        return (getDistance())
                * WHEEL_RADIUS;
    }

    public double getDistance() {
        // _drivePositionRotations.refresh();
        return _drivePositionRotations.getValue() * (Math.PI * 2.0) / (_isLowGear ? Constants.SwerveModule.GEAR_RATIO_DRIVE_LOW : Constants.SwerveModule.GEAR_RATIO_DRIVE_HIGH);
    }

    public void resetEncoders() {
        _driveMotor.setPosition(0);
        _steeringMotor.setPosition(0);
    }

    public void updateDashboard() {
        SmartDashboard.putNumber("/moduleAngle", getEncoderAngleDouble());
        SmartDashboard.putNumber("/driveRPM", getDriveRPM());
        SmartDashboard.putNumber("/wantedSpeed", _wantedSpeed);
        SmartDashboard.putNumber("/wheelVelocity", getWheelVelocity());
        SmartDashboard.putNumber("/wheelAngularVelocity", getWheelAngularVelocity());
        SmartDashboard.putNumber("/driveVoltage", _driveMotor.getSupplyVoltage().getValue());
        SmartDashboard.putNumber("/steerVoltage", _steeringMotor.getSupplyVoltage().getValue());
        SmartDashboard.putBoolean("/isLowGear", _isLowGear);
        SmartDashboard.putNumber("/driveSupplyCurrent", getDriveMotorCurrent());
        SmartDashboard.putNumber("/steerSupplyCurrent", _steeringMotor.getSupplyCurrent().getValue());
        SmartDashboard.putNumber("/drivePosition", _drivePositionRotations.getValue());
        SmartDashboard.putNumber("/distance", getDistance());
    }

    public static class ModuleConfiguration {
        public String moduleName = "";

        public Translation2d position = new Translation2d();

        public double encoderOffset = 0.0;
        public boolean encoderInverted = false;

        public String canBus = "CANivore";
    }

    // public enum ControlState {
    // OFF(0),
    // STATE_CONTROL(1),
    // CHARACTERIZATION(2);
    // private final int _value;

    // ControlState(int value) {
    // _value = value;
    // }

    // public int getValue() {
    // return _value;
    // }
    // }

}
