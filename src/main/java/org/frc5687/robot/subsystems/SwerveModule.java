/* Team 5687  */
package org.frc5687.robot.subsystems;

import static org.frc5687.robot.Constants.SwerveModule.WHEEL_RADIUS;
import static org.frc5687.robot.Constants.SwerveModule.kDt;

import org.frc5687.lib.drivers.OutliersTalon;
import org.frc5687.robot.Constants;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Swerve Module Code Created in the shadow of
// the death of diffy swerve by Linus Krenkel
// using VelocityFOC and PositionVoltage 
public class SwerveModule {

    public static class ModuleConfiguration {
        public String moduleName = "";

        public Translation2d position = new Translation2d();

        public double encoderOffset = 0.0;
        public boolean encoderInverted = false;

        public String canBus = "CANivore";
    }

    private final OutliersTalon _driveMotor;
    private final OutliersTalon _steeringMotor;
    private final CANcoder _encoder;

    private Translation2d _modulePosition;

    private SwerveModuleState _goal;

    private final BaseStatusSignal[] _signals = new BaseStatusSignal[4];
    private StatusSignal<Double> _driveVelocityRotationsPerSec;
    private StatusSignal<Double> _drivePositionRotations;
    private StatusSignal<Double> _steeringVelocityRotationsPerSec;
    private StatusSignal<Double> _steeringPositionRotations;

    private VelocityTorqueCurrentFOC _velocityTorqueCurrentFOC;
    private MotionMagicExpoTorqueCurrentFOC _angleTorqueExpo;
    private MotionMagicTorqueCurrentFOC _angleTorque;

    private SwerveModulePosition _internalState = new SwerveModulePosition();

    private double _rotPerMet;

    private double _wantedSpeed;
    private double _stateMPS;

    private boolean _isLowGear;
    private String _moduleName;


    public SwerveModule(
            SwerveModule.ModuleConfiguration config,
            int steeringMotorID,
            int driveMotorID,
            int encoderPort) {
        /* Control Requests */
        // Driving Torque Velocity
        _velocityTorqueCurrentFOC = new VelocityTorqueCurrentFOC(0).withOverrideCoastDurNeutral(true);
        // Steering Torque Position with exponential curve
        _angleTorqueExpo = new MotionMagicExpoTorqueCurrentFOC(0);
        _angleTorque = new MotionMagicTorqueCurrentFOC(0).withOverrideCoastDurNeutral(true);
        /* Motor Setup */
        _driveMotor = new OutliersTalon(driveMotorID, config.canBus, "Drive");
        _driveMotor.configure(Constants.SwerveModule.CONFIG);
        _driveMotor.configureClosedLoop(Constants.SwerveModule.DRIVE_CONTROLLER_CONFIG);

        _steeringMotor = new OutliersTalon(steeringMotorID, config.canBus, "Steer");
        _steeringMotor.configure(Constants.SwerveModule.STEER_CONFIG);
        _steeringMotor.configureClosedLoop(Constants.SwerveModule.STEER_CONTROLLER_CONFIG);

        // MotionMagicConfigs motionMagicConfigs= new MotionMagicConfigs();
        // motionMagicConfigs.MotionMagicCruiseVelocity = 0; 
        // motionMagicConfigs.MotionMagicAcceleration = 0;
        // motionMagicConfigs.MotionMagicCruiseVelocity  = 100.0;
        // motionMagicConfigs.MotionMagicAcceleration = motionMagicConfigs.MotionMagicCruiseVelocity / 0.100;
        // motionMagicConfigs.MotionMagicExpo_kV = 0.012;
        //  motionMagicConfigs.MotionMagicExpo_kA = 0.001;

        // _steeringMotor.getConfigurator().apply(motionMagicConfigs);

        _isLowGear = false;

        _encoder = new CANcoder(encoderPort, config.canBus);
        CANcoderConfiguration CANfig = new CANcoderConfiguration();
        // set units of the CANCoder to radians, with velocity being radians per second
        CANfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        CANfig.MagnetSensor.MagnetOffset = config.encoderOffset;
        CANfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;

        _encoder.getConfigurator().apply(CANfig);

        _goal = new SwerveModuleState(0.0, getCanCoderAngle());

        FeedbackConfigs feedback = new FeedbackConfigs();
        feedback.FeedbackRemoteSensorID = encoderPort;
        feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        // feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        feedback.RotorToSensorRatio = Constants.SwerveModule.GEAR_RATIO_STEER;


        _steeringMotor.configureFeedback(feedback);

        _modulePosition = config.position;
        _rotPerMet = 1 / (2 * Math.PI * Constants.SwerveModule.WHEEL_RADIUS);

        initializeSignals();

        // _controlState = ControlState.OFF;
        _moduleName = config.moduleName;
        System.out.println(_moduleName + " Module has been constructed!!");
    }

    private void initializeSignals() {
        _drivePositionRotations = _driveMotor.getPosition();
        _driveVelocityRotationsPerSec = _driveMotor.getVelocity();
        _steeringPositionRotations = _encoder.getPosition();
        _steeringVelocityRotationsPerSec = _encoder.getVelocity();

        _driveMotor.getFault_Hardware().setUpdateFrequency(4, 0.04);
        _driveVelocityRotationsPerSec.setUpdateFrequency(1 / kDt);
        _drivePositionRotations.setUpdateFrequency(1 / kDt);

        _steeringMotor.getFault_Hardware().setUpdateFrequency(4, 0.04);
        _steeringVelocityRotationsPerSec.setUpdateFrequency(1 / kDt);
        _steeringPositionRotations.setUpdateFrequency(1 / kDt);

        _signals[0] = _driveVelocityRotationsPerSec;
        _signals[1] = _drivePositionRotations;
        _signals[2] = _steeringVelocityRotationsPerSec;
        _signals[3] = _steeringPositionRotations;
    }

    public void setControlRequestUpdateFrequency(double updateFreqHz) {
        _velocityTorqueCurrentFOC.UpdateFreqHz = updateFreqHz;
        _angleTorque.UpdateFreqHz = updateFreqHz;
        _angleTorqueExpo.UpdateFreqHz = updateFreqHz;
    }

    public void refreshSignals() {
        _drivePositionRotations.refresh();
        _driveVelocityRotationsPerSec.refresh();
        _steeringPositionRotations.refresh();
        _steeringVelocityRotationsPerSec.refresh();
    }

    public void shiftDown() {
        if (!_isLowGear) {
            setLowGear();
            transformEncoderFromHighGearToLowGear();
        }
    }

    public void shiftUp() {
        if (_isLowGear) {
            setHighGear();
            transformEncoderFromLowGearToHighGear();
        }
    }

    private SwerveModulePosition calculatePosition() {
        double currentEncoderRotations = BaseStatusSignal.getLatencyCompensatedValue(_drivePositionRotations, _driveVelocityRotationsPerSec);
        double distanceMeters = currentEncoderRotations * 2.0 * Math.PI / getGearRatio() * Constants.SwerveModule.WHEEL_RADIUS ;
        double angle_rot = BaseStatusSignal.getLatencyCompensatedValue(_steeringPositionRotations,
                _steeringVelocityRotationsPerSec);

        _internalState.distanceMeters = distanceMeters;
        _internalState.angle = Rotation2d.fromRotations(angle_rot);

        return _internalState;
    }

    private double getGearRatio() {
        return _isLowGear ? Constants.SwerveModule.GEAR_RATIO_DRIVE_LOW : Constants.SwerveModule.GEAR_RATIO_DRIVE_HIGH;
    }

    public BaseStatusSignal[] getSignals() {
        return _signals;
    }

    public void setIdealState(SwerveModuleState state) {
        state = SwerveModuleState.optimize(state, _internalState.angle);
        _goal = state;
        // if (Math.abs(state.speedMetersPerSecond) < Constants.SwerveModule.IDLE_MPS_LIMIT) {
            // setModuleState(new SwerveModuleState(0.0, _goal.angle));
        // } else {
        setModuleState(_goal);
        // }
    }

    private double calculateWantedSpeed(SwerveModuleState state) {
        return state.speedMetersPerSecond * getGearRatio() * _rotPerMet;
    }

    public void setModuleState(SwerveModuleState state) {
        _stateMPS = state.speedMetersPerSecond;
        _wantedSpeed = calculateWantedSpeed(state);

        // skew correction logic, team 900 paper, used in CTRE Swerve
        double steerMotorError = state.angle.minus(getCanCoderAngle()).getRadians();
        double cosineScalar = Math.cos(steerMotorError);
        cosineScalar = Math.max(cosineScalar, 0.0); // Ensure it does not invert drive
        _wantedSpeed *= cosineScalar;

        double position = state.angle.getRotations();

        _driveMotor.setControl(_velocityTorqueCurrentFOC.withVelocity(_wantedSpeed));
        _steeringMotor.setPositionVoltage(position);
        // This is too slow for me :(
        // Use new torque exponential curve
        // _steeringMotor.setControl(_angleTorqueExpo.withPosition(position));
        // _steeringMotor.setControl(_angleTorque.withPosition(position));

        // For debugging
        SmartDashboard.putNumber("/actualSpeed", _driveMotor.getVelocity().getValue());
        SmartDashboard.putNumber("/wantedPosition", position);
        SmartDashboard.putNumber("/cosineScalar", cosineScalar);
    }
    private void transformEncoderFromHighGearToLowGear() {
        refreshSignals();
        double currentEncoderRotations = BaseStatusSignal.getLatencyCompensatedValue(_drivePositionRotations, _driveVelocityRotationsPerSec);
        _driveMotor.setPosition(currentEncoderRotations * Constants.SwerveModule.GEAR_RATIO_DRIVE_LOW / Constants.SwerveModule.GEAR_RATIO_DRIVE_HIGH);
    }

    private void transformEncoderFromLowGearToHighGear() {
        refreshSignals();
        double currentEncoderRotations = BaseStatusSignal.getLatencyCompensatedValue(_drivePositionRotations, _driveVelocityRotationsPerSec);
        _driveMotor.setPosition(currentEncoderRotations * Constants.SwerveModule.GEAR_RATIO_DRIVE_HIGH / Constants.SwerveModule.GEAR_RATIO_DRIVE_LOW);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getWheelVelocity(), getCanCoderAngle());
    }

    public double getDriveMotorVoltage() {
        return _driveMotor.getSupplyVoltage().getValue();
    }

    public double getDriveMotorCurrent() {
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

    public void setToBrake() {
        _driveMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void setToCoast() {
        _driveMotor.setNeutralMode(NeutralModeValue.Coast);
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

    private void setLowGear() {
        _isLowGear = true;
        _velocityTorqueCurrentFOC = _velocityTorqueCurrentFOC.withSlot(0);
    }

    private void setHighGear() {
        _isLowGear = false;
        _velocityTorqueCurrentFOC = _velocityTorqueCurrentFOC.withSlot(1);
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

    public double getWantedSpeed() {
        return _wantedSpeed;
    }

    public double getStateMPS() {
        return _stateMPS;
    }

    public double getWheelAngularVelocity() {
        return Units.rotationsPerMinuteToRadiansPerSecond(getDriveRPM() / getGearRatio());
    }

    public Translation2d getModuleLocation() {
        return _modulePosition;
    }

    public SwerveModulePosition getPosition() {
        return calculatePosition();
    }

    public double getWheelDistance() {
        return getDistance() * WHEEL_RADIUS;
    }

    public double getDistance() {
        return _drivePositionRotations.getValue() * (Math.PI * 2.0) / (getGearRatio());
    }

    public void resetEncoders() {
        _driveMotor.setPosition(0);
        _steeringMotor.setPosition(0);
    }

    public void applyCharacterization(Rotation2d steerTarget, double voltage){
        double angle = steerTarget.getRotations();
        _steeringMotor.setControl(_angleTorqueExpo.withPosition(angle));
        setDriveMotorVoltage(voltage);
    }

    public void setDriveMotorVoltage(double voltage) {
        _driveMotor.setVoltage(voltage);
    }

    public void setSteerMotorVoltage(double voltage) {
        _steeringMotor.setVoltage(voltage);
    }

    public void updateDashboard() {
        SmartDashboard.putNumber(_moduleName + "/moduleAngle", getEncoderAngleDouble());
        SmartDashboard.putNumber(_moduleName + "/driveRPM", getDriveRPM());

        SmartDashboard.putBoolean(_moduleName + "/isLowGear", _isLowGear);
        SmartDashboard.putNumber(_moduleName + "/wantedSpeed", _wantedSpeed);

        SmartDashboard.putNumber(_moduleName + "/wantedWheelVelocity", _stateMPS);
        SmartDashboard.putNumber(_moduleName + "/wheelVelocity", getWheelVelocity());
        SmartDashboard.putNumber(_moduleName + "/wheelAngularVelocity", getWheelAngularVelocity());
        SmartDashboard.putNumber(_moduleName + "/driveVoltage", _driveMotor.getSupplyVoltage().getValue());
        SmartDashboard.putNumber(_moduleName + "/steerVoltage", _steeringMotor.getSupplyVoltage().getValue());

        SmartDashboard.putNumber(_moduleName + "/driveSupplyCurrent", getDriveMotorCurrent());
        SmartDashboard.putNumber(_moduleName + "/steerSupplyCurrent", _steeringMotor.getSupplyCurrent().getValue());

        SmartDashboard.putNumber(_moduleName + "/drivePosition", _drivePositionRotations.getValue());
        SmartDashboard.putNumber(_moduleName + "/distance", getDistance());
    }
}
