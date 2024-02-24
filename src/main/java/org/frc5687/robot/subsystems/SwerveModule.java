/* Team 5687  */
package org.frc5687.robot.subsystems;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static org.frc5687.robot.Constants.SwerveModule.WHEEL_RADIUS;
import static org.frc5687.robot.Constants.SwerveModule.kDt;

import org.frc5687.lib.drivers.OutliersTalon;
import org.frc5687.robot.Constants;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

// Swerve Module Code Created in the shadow of
// the death of diffy swerve by Linus Krenkel
// using VelocityFOC and PositionVoltage 
public class SwerveModule extends SubsystemBase{

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

    private SwerveModulePosition _internalState = new SwerveModulePosition();

    private double _rotPerMet;

    private double _wantedSpeed;
    private double _stateMPS;

    private boolean _isLowGear;
    private String _moduleName;

    // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
    private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
    // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
    private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
    // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
    private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));

        // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
    private final MutableMeasure<Voltage> m_appliedVoltageSteer = mutable(Volts.of(0));
    // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
    private final MutableMeasure<Angle> m_angle = mutable(Rotations.of(0));
    // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
    private final MutableMeasure<Velocity<Angle>> m_velocitySteer = mutable(RotationsPerSecond.of(0));

    private final SysIdRoutine m_sysIdRoutine;
    public SwerveModule(
            SwerveModule.ModuleConfiguration config,
            int steeringMotorID,
            int driveMotorID,
            int encoderPort) {

        _driveMotor = new OutliersTalon(driveMotorID, config.canBus, "Drive");
        _driveMotor.configure(Constants.SwerveModule.CONFIG);
        _driveMotor.configureClosedLoop(Constants.SwerveModule.DRIVE_CONTROLLER_CONFIG);

        _steeringMotor = new OutliersTalon(steeringMotorID, config.canBus, "Steer");
        _steeringMotor.configure(Constants.SwerveModule.STEER_CONFIG);
        _steeringMotor.configureClosedLoop(Constants.SwerveModule.STEER_CONTROLLER_CONFIG);
        _isLowGear = false;

        _velocityTorqueCurrentFOC = new VelocityTorqueCurrentFOC(0.0, 0.0, 0, 0, true, false, false);
        _velocityTorqueCurrentFOC.OverrideCoastDurNeutral = true;
        // _positionVoltage = new PositionVoltage(0.0);

        _encoder = new CANcoder(encoderPort, config.canBus);
        CANcoderConfiguration CANfig = new CANcoderConfiguration();
        // set units of the CANCoder to radians, with velocity being radians per second
        CANfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        CANfig.MagnetSensor.MagnetOffset = config.encoderOffset;
        CANfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;

        _encoder.getConfigurator().apply(CANfig);

        _velocityTorqueCurrentFOC = new VelocityTorqueCurrentFOC(0.0, 0.0, 0, 0, true, false, false);
        _velocityTorqueCurrentFOC.OverrideCoastDurNeutral = true;

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
        m_sysIdRoutine = new SysIdRoutine(
          // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
          new SysIdRoutine.Config(),
          new SysIdRoutine.Mechanism(
              // Tell SysId how to plumb the driving voltage to the motors.
              (Measure<Voltage> volts) -> {
                _driveMotor.setVoltage(volts.in(Volts));
                _steeringMotor.setVoltage(volts.in(Volts));
              },
              // Tell SysId how to record a frame of data for each motor on the mechanism being
              // characterized.
              log -> {
                // Record a frame for the left motors.  Since these share an encoder, we consider
                // the entire group to be one motor.
                log.motor(_moduleName + "_drive")
                    .voltage(
                        m_appliedVoltage.mut_replace(_driveMotor.getMotorVoltage().getValueAsDouble(), Volts))
                    .linearPosition(m_distance.mut_replace(getDistance(), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(getWheelVelocity(), MetersPerSecond));
                // Record a frame for the right motors.  Since these share an encoder, we consider
                // the entire group to be one motor.
                log.motor(_moduleName + "_steer")
                    .voltage(
                        m_appliedVoltageSteer.mut_replace(
                            _steeringMotor.getMotorVoltage().getValueAsDouble(), Volts))
                    .angularPosition(m_angle.mut_replace(_steeringMotor.getRotorPosition().getValueAsDouble(), Rotations))
                    .angularVelocity(
                        m_velocitySteer.mut_replace(_steeringMotor.getRotorVelocity().getValueAsDouble(), RotationsPerSecond));
              },
              // Tell SysId to make generated commands require this subsystem, suffix test state in
              // WPILog with this subsystem's name ("drive")
              this));
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

        double distanceMeters = currentEncoderRotations * 2.0 * 3.14159265 / getGearRatio() * Constants.SwerveModule.WHEEL_RADIUS ;
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
        state = SwerveModuleState.optimize(state, getCanCoderAngle());
        _goal = state;
        setModuleState(_goal);
    }

    private double calculateWantedSpeed(SwerveModuleState state) {
        return state.speedMetersPerSecond * getGearRatio() * _rotPerMet;
    }

    public void setModuleState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < Constants.SwerveModule.IDLE_MPS_LIMIT) {
            stopAll();
        } else {
            _stateMPS = state.speedMetersPerSecond;
            _wantedSpeed = calculateWantedSpeed(state);
            double position = state.angle.getRotations();

            _driveMotor.setControl(_velocityTorqueCurrentFOC.withVelocity(_wantedSpeed));
            _steeringMotor.setPositionVoltage(position);
            SmartDashboard.putNumber("/actualSpeed", _driveMotor.getVelocity().getValue());
            SmartDashboard.putNumber("/wantedPosition", position);
        }
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
        DriverStation.reportError("Setting Low Gear", false);
    }

    private void setHighGear() {
        _isLowGear = false;
        _velocityTorqueCurrentFOC = _velocityTorqueCurrentFOC.withSlot(1);
        DriverStation.reportError("Setting High Gear", false);
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
        return Units.rotationsPerMinuteToRadiansPerSecond(getDriveRPM()   / getGearRatio());
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

    public void updateDashboard() {
        SmartDashboard.putNumber(_moduleName + "/moduleAngle", getEncoderAngleDouble());
        SmartDashboard.putNumber(_moduleName + "/driveRPM", getDriveRPM());

        SmartDashboard.putBoolean(_moduleName + "/isLowGear", _isLowGear);
        SmartDashboard.putNumber(_moduleName + "/wantedSpeed", _wantedSpeed);

        SmartDashboard.putNumber(_moduleName + "/wheelVelocity", getWheelVelocity());
        SmartDashboard.putNumber(_moduleName + "/wheelAngularVelocity", getWheelAngularVelocity());
        SmartDashboard.putNumber(_moduleName + "/driveVoltage", _driveMotor.getSupplyVoltage().getValue());
        SmartDashboard.putNumber(_moduleName + "/steerVoltage", _steeringMotor.getSupplyVoltage().getValue());

        SmartDashboard.putNumber(_moduleName + "/driveSupplyCurrent", getDriveMotorCurrent());
        SmartDashboard.putNumber(_moduleName + "/steerSupplyCurrent", _steeringMotor.getSupplyCurrent().getValue());

        SmartDashboard.putNumber(_moduleName + "/drivePosition", _drivePositionRotations.getValue());
        SmartDashboard.putNumber(_moduleName + "/distance", getDistance());
    }

    public static class ModuleConfiguration {
        public String moduleName = "";

        public Translation2d position = new Translation2d();

        public double encoderOffset = 0.0;
        public boolean encoderInverted = false;

        public String canBus = "CANivore";
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.dynamic(direction);
    }
}
