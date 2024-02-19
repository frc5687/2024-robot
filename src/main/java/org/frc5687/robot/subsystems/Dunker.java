package org.frc5687.robot.subsystems;

import org.frc5687.lib.drivers.OutliersTalon;
import org.frc5687.lib.sensors.ProximitySensor;
import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotMap;
import org.frc5687.robot.util.OutliersContainer;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class Dunker extends OutliersSubsystem{

    public enum DunkerState { 
        UNKNOWN(0),
        STOWED(1),
        PREPARED_FOR_NOTE(2),
        NOTE_IN_DUNKER(3),
        READY_TO_DUNK(4),
        DUNKED_NOTE(5);

        private final int _value;
        DunkerState (int value) { 
            _value = value; 
        }

        public int getValue() { 
            return _value; 
        }
    }

    private OutliersTalon _dunkerArmTalon;
    private DutyCycleEncoder _dunkerAbsEncoder;
    private ProximitySensor _dunkerProx;
    private DunkerState _dunkerState;

    public Dunker(OutliersContainer container) {
        super(container);

        _dunkerArmTalon = new OutliersTalon(RobotMap.CAN.TALONFX.DUNKER_ARM, Constants.Dunker.CAN_BUS, "Dunker Arm");
        _dunkerArmTalon.configure(Constants.Dunker.CONFIG);

        _dunkerArmTalon.configureClosedLoop(Constants.Dunker.CLOSED_LOOP_CONFIG);

        _dunkerProx = new ProximitySensor(RobotMap.DIO.DUNKER_PROXIMITY_SENSOR);

        _dunkerAbsEncoder = new DutyCycleEncoder(RobotMap.DIO.DUNKER_ABS_ENCODER);
        _dunkerAbsEncoder.setDistancePerRotation(Math.PI * 2.0);  
        _dunkerState = DunkerState.UNKNOWN;
    }

    public void setDunkerAngle(double angle) {
        _dunkerArmTalon.setMotionMagic(angle * Constants.Shooter.DUNKER_GEAR_RATIO);
    }

    public double getDunkerAngle() {
        return OutliersTalon.rotationsToRadians(_dunkerArmTalon.getPosition().getValueAsDouble(), Constants.Shooter.DUNKER_GEAR_RATIO);
    }

    public double getDunkerAbsAngle() {
        return _dunkerAbsEncoder.getAbsolutePosition();
    }

    public void resetMotorEncoderFromAbs() {
        double rotations = OutliersTalon.radiansToRotations(getDunkerAbsAngle(), Constants.Shooter.DUNKER_GEAR_RATIO);
        _dunkerArmTalon.setPosition(rotations);
    }

    // in radians
    public boolean isAtAngle(double angle) {
        return Math.abs(getDunkerAngle() - angle) > Constants.Dunker.ANGLE_TOLERANCE;
    }
    
    public boolean isNoteInDunker() {
        return _dunkerProx.get();
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
        // Check if the abs encoder and the relative encoder are not in sync, if so reset relative to abs
        if (Math.abs(getDunkerAbsAngle() - getDunkerAngle()) > Constants.Dunker.ANGLE_SYNC_TOLERANCE) {
            resetMotorEncoderFromAbs();
        }
    }

    @Override
    public void updateDashboard() {
        metric("Dunker Angle", getDunkerAngle());
        metric("Note in Dunker", isNoteInDunker());
    }
}
