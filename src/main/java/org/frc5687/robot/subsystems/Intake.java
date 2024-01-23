
package org.frc5687.robot.subsystems;

import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotMap;
import org.frc5687.robot.util.OutliersContainer;
import org.frc5687.lib.drivers.OutliersTalon;

public class Intake extends OutliersSubsystem {

    private final OutliersTalon _talon;

    public Intake(OutliersContainer container) {
        super(container);
        _talon = new OutliersTalon(RobotMap.CAN.TALONFX.INTAKE, Constants.Intake.CAN_BUS, "Intake");
        
    }

    public void setSpeed(double intakeSpeed) {
        _talon.setPercentOutput(intakeSpeed);
    }

    @Override
    public void updateDashboard() {
        
    }
}