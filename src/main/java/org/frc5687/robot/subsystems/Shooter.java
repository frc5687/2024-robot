package org.frc5687.robot.subsystems;

import org.frc5687.lib.drivers.OutliersTalon;
import org.frc5687.robot.util.OutliersContainer;
import org.frc5687.robot.RobotMap;

public class Shooter extends OutliersSubsystem {
    public OutliersTalon _leftTalon;
    public OutliersTalon _rightTalon;
    public Shooter(OutliersContainer container) {
        super(container);
        _leftTalon = new OutliersTalon(RobotMap.CAN.TALONFX.LEFT_SHOOTER, "rio", "Left Shooter");
        _rightTalon = new OutliersTalon(RobotMap.CAN.TALONFX.RIGHT_SHOOTER, "rio", "Right Shooter");
    }

    public void setSpeed(double speed) {
        _leftTalon.setPercentOutput(-speed);
        _rightTalon.setPercentOutput(speed);
    }

    public void updateDashboard() {}
}
