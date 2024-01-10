package org.frc5687.robot.subsystems;

import org.frc5687.lib.drivers.OutliersTalon;
import org.frc5687.robot.util.OutliersContainer;

public class Shooter {
    public OutliersTalon _leftTalon;
    public OutliersTalon _rightTalon;
    public Shooter(OutliersContainer container) {
        // super(container);
        _leftTalon = new OutliersTalon(0, null, null);
        _rightTalon = new OutliersTalon(0, null, null);
    }

    public void setSpeed(double speed) {
        _leftTalon.setPercentOutput(-speed);
        _rightTalon.setPercentOutput(speed);
    }

    public void updateDashboard() {}
}
