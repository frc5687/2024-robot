
package org.frc5687.robot.subsystems;

import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotMap;
import org.frc5687.robot.util.OutliersContainer;
import org.frc5687.lib.drivers.OutliersTalon;
import org.frc5687.lib.sensors.ProximitySensor;

public class Intake extends OutliersSubsystem {

    private final OutliersTalon _talon;
    private final ProximitySensor _proximitySensor;
    public Intake(OutliersContainer container) {
        super(container);
        _talon = new OutliersTalon(RobotMap.CAN.TALONFX.INTAKE, Constants.Intake.CAN_BUS, "Intake");
        _talon.configure(Constants.Intake.CONFIG);
        _proximitySensor = new ProximitySensor(RobotMap.DIO.DONUT_SENSOR);
    }
    
    public boolean isDonutDetected(){
        return _proximitySensor.get();
    }

    public void setSpeed(double intakeSpeed) {
        _talon.setPercentOutput(intakeSpeed);
    }

    @Override
    public void updateDashboard() {
        metric("Note in Chamber", isDonutDetected());
    }
    
}