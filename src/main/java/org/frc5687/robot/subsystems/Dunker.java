package org.frc5687.robot.subsystems;

import org.frc5687.lib.drivers.OutliersTalon;
import org.frc5687.lib.sensors.ProximitySensor;
import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotMap;
import org.frc5687.robot.util.OutliersContainer;

public class Dunker extends OutliersSubsystem{
    private OutliersTalon _dunkerArmTalon;
    private ProximitySensor _dunkerProx;

    public Dunker(OutliersContainer container) {
        super(container);

        _dunkerArmTalon = new OutliersTalon(RobotMap.CAN.TALONFX.DUNKER_ARM, Constants.Dunker.CAN_BUS, "Dunker Arm");

        _dunkerArmTalon.configure(Constants.Dunker.CONFIG);
        _dunkerArmTalon.configureClosedLoop(Constants.Dunker.CLOSED_LOOP_CONFIG);

        _dunkerProx = new ProximitySensor(RobotMap.DIO.DUNKER_PROXIMITY_SENSOR);
    }

    public void setDunkerAngle(double angle) {
        _dunkerArmTalon.setMotionMagic(angle * Constants.Shooter.DUNKER_GEAR_RATIO);
    }

    public double getDunkerAngle() {
        return OutliersTalon.rotationsToRadians(_dunkerArmTalon.getPosition().getValueAsDouble(), Constants.Shooter.DUNKER_GEAR_RATIO);
    }
    
    public boolean isNoteInDunker() {
        return _dunkerProx.get();
    }

    @Override
    public void updateDashboard() {
        metric("Dunker Angle", getDunkerAngle());
        metric("Note in Dunker", isNoteInDunker());
    }
}
