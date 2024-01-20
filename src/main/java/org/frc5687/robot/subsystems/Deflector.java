package org.frc5687.robot.subsystems;

import org.frc5687.lib.drivers.OutliersTalon;
import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotMap;
import org.frc5687.robot.util.OutliersContainer;

public class Deflector extends OutliersSubsystem {

    private OutliersTalon _talon;
    private double _targetAngle = 0;

    public Deflector(OutliersContainer container) {
        super(container);
        _talon = new OutliersTalon(RobotMap.CAN.TALONFX.DEFLECTOR, Constants.Deflector.CAN_BUS, "deflector");
        _talon.configure(Constants.Deflector.CONFIG);
        _talon.configureClosedLoop(Constants.Deflector.CLOSED_LOOP_CONFIG);
    }

    public void setTargetAngle(double angle) {
        _targetAngle = angle;
    }

    public double getTargetAngle() {
        return _targetAngle;
    }

    public void setMotionMagic(double position) {
        _talon.setMotionMagic(position);
    }

    @Override
    public void updateDashboard() {

    }
}
