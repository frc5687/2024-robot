
package org.frc5687.robot.subsystems;

import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotMap;
import org.frc5687.robot.util.OutliersContainer;
import org.frc5687.lib.drivers.OutliersTalon;
import org.frc5687.lib.sensors.ProximitySensor;

public class Intake extends OutliersSubsystem {

    private final OutliersTalon _talon;
    private final ProximitySensor _top;
    private final ProximitySensor _bottom;
    private boolean _autoIntakeFlag = false;

    public Intake(OutliersContainer container) {
        super(container);
        _talon = new OutliersTalon(RobotMap.CAN.TALONFX.INTAKE, Constants.Intake.CAN_BUS, "Intake");
        _talon.configure(Constants.Intake.CONFIG);
        _top = new ProximitySensor(RobotMap.DIO.TOP_DONUT_SENSOR);
        _bottom = new ProximitySensor(RobotMap.DIO.BOTTOM_DONUT_SENSOR);
    }
    
    public boolean isTopDetected(){
        return _top.get();
    }

    public boolean isBottomDetected() {
        return _bottom.get();
    }

    public void setSpeed(double intakeSpeed) {
        _talon.setPercentOutput(intakeSpeed);
    }

    public void setAutoIntakeFlag(boolean flag) {
        _autoIntakeFlag = flag;
    }

    public boolean getAutoIntakeFlag() {
        return _autoIntakeFlag;
    }

    @Override
    public void updateDashboard() {
        metric("Top Detected", isTopDetected());
        metric("Bottom Detected", isBottomDetected());
        metric("AutoIntake Flag", getAutoIntakeFlag());
    }
    
}