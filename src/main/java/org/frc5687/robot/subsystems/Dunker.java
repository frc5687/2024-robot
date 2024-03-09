package org.frc5687.robot.subsystems;

import org.frc5687.lib.drivers.OutliersTalon;
import org.frc5687.lib.sensors.ProximitySensor;
import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotMap;
import org.frc5687.robot.util.Helpers;
import org.frc5687.robot.util.OutliersContainer;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class Dunker extends OutliersSubsystem {

    public enum DunkerState {
        UNKNOWN(0),
        STOWING(1),
        STOWED(2),
        PREPARED_FOR_NOTE(3),
        NOTE_IN_DUNKER(4),
        READY_TO_DUNK(5),
        DUNKED_NOTE(6);

        private final int _value;

        DunkerState(int value) {
            _value = value;
        }

        public int getValue() {
            return _value;
        }
    }

    private OutliersTalon _dunkerArmTalon;
    private OutliersTalon _dunkerDriveTalon;
    private DutyCycleEncoder _dunkerAbsEncoder;
    private ProximitySensor _dunkerProx;
    private DunkerState _dunkerState;
    private int _tickCounter = 0;
    private double _targetRPM = 0;

    public Dunker(OutliersContainer container) {
        super(container);

        _dunkerArmTalon = new OutliersTalon(RobotMap.CAN.TALONFX.DUNKER_ARM, Constants.Dunker.CAN_BUS, "Dunker Arm");
        _dunkerArmTalon.configure(Constants.Dunker.ARM_CONFIG);
        _dunkerArmTalon.configureClosedLoop(Constants.Dunker.ARM_CLOSED_LOOP_CONFIG);

        _dunkerDriveTalon = new OutliersTalon(RobotMap.CAN.TALONFX.DUNKER_DRIVE, Constants.Dunker.CAN_BUS, "Dunker Drive");
        _dunkerDriveTalon.configure(Constants.Dunker.DRIVE_CONFIG);
        _dunkerDriveTalon.configureClosedLoop(Constants.Dunker.DRIVE_CLOSED_LOOP_CONFIG);


        _dunkerProx = new ProximitySensor(RobotMap.DIO.DUNKER_PROXIMITY_SENSOR);

        _dunkerAbsEncoder = new DutyCycleEncoder(RobotMap.DIO.DUNKER_ABS_ENCODER);
        _dunkerAbsEncoder.setDistancePerRotation(Math.PI * 2.0);
        _dunkerState = DunkerState.STOWED;

        // Only check every 10 ticks (200ms);
        _tickCounter = 0;
    }

    public void setDunkerAngle(double angle) {
        _dunkerArmTalon.setMotionMagic(OutliersTalon.radiansToRotations(angle, Constants.Dunker.DUNKER_ARM_GEAR_RATIO));
    }

    public void disable() {
        _dunkerArmTalon.disable();
    }

    public double getDunkerAngle() {
        return OutliersTalon.rotationsToRadians(_dunkerArmTalon.getPosition().getValueAsDouble(),
                Constants.Dunker.DUNKER_ARM_GEAR_RATIO);
    }

    public double getDunkerAbsAngleRadians() {
        return Helpers.angleWrap(_dunkerAbsEncoder.getDistance(), true);
    }

    public void resetMotorEncoderFromAbs() {
        double rotations = OutliersTalon.radiansToRotations(getDunkerAbsAngleRadians(),
                Constants.Dunker.DUNKER_ARM_GEAR_RATIO);
        _dunkerArmTalon.setPosition(rotations);
    }

    // in radians
    public boolean isAtAngle(double angle) {
        return Math.abs(getDunkerAngle() - angle) < Constants.Dunker.ANGLE_TOLERANCE;
    }

    public boolean isNoteInDunker() {
        return _dunkerProx.get();
    }

    public void setToStop() {
        _dunkerDriveTalon.setVelocity(0);
        _targetRPM = 0;
    }

    public void setToHandoffRPM() {
        _dunkerDriveTalon.setVelocity(Constants.Dunker.DUNKER_IN_RPM);
        _targetRPM = Constants.Dunker.DUNKER_IN_RPM;
    }

    public void setToDunkRPM() {
        _dunkerDriveTalon.setVelocity(Constants.Dunker.DUNKER_OUT_RPM);
        _targetRPM = Constants.Dunker.DUNKER_OUT_RPM;
    }

    public DunkerState getDunkerState() {
        return _dunkerState;
    }

    public void setDunkerState(DunkerState state) {
        _dunkerState = state;
    }

    @Override
    public void periodic() {
        super.periodic();
        // Check if the abs encoder and the relative encoder are not in sync, if so
        // reset relative to abs
        _tickCounter++;
        if (Math.abs(getDunkerAbsAngleRadians() - getDunkerAngle()) > Constants.Dunker.ANGLE_SYNC_TOLERANCE
                && _tickCounter % 10 == 0) {
            resetMotorEncoderFromAbs();
            _tickCounter = 0;
        }
    }

    @Override
    public void updateDashboard() {
        metric("Dunker Angle", getDunkerAngle());
        metric("Note in Dunker", isNoteInDunker());
        metric("Dunker absolute angle radians", getDunkerAbsAngleRadians());
        metric("Dunker target angle", _dunkerArmTalon.getClosedLoopReference().getValue());
        metric("Dunker Drive Target RPM", _targetRPM);
        metric("Dunker RPM", OutliersTalon.rotationsPerSecToRPM(_dunkerDriveTalon.getVelocity().getValue(), 1));
        metric("State", _dunkerState.name());
    }
}
