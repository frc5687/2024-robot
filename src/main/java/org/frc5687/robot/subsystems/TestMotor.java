
package org.frc5687.robot.subsystems;

import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotMap;
import org.frc5687.robot.util.OutliersContainer;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import org.frc5687.lib.drivers.OutliersTalon;
import org.frc5687.lib.drivers.OutliersTalon.Configuration;

public class TestMotor extends OutliersSubsystem {

    private final OutliersTalon _goodTalon;
    private final OutliersTalon _badTalon;
    public TestMotor(OutliersContainer container) {
        super(container);
        _goodTalon = new OutliersTalon(0, "CANivore", "Test Good Kraken");
        _badTalon = new OutliersTalon(1, "CANivore", "Test Bad Kraken");
        Configuration configuration = new Configuration();
        configuration.TIME_OUT = 0.1;

        configuration.NEUTRAL_MODE = NeutralModeValue.Coast;
        configuration.INVERTED = InvertedValue.CounterClockwise_Positive;

        configuration.MAX_VOLTAGE = 12.0;

        configuration.MAX_STATOR_CURRENT = Double.POSITIVE_INFINITY;
        configuration.MAX_CURRENT = Double.POSITIVE_INFINITY;
        // configuration.ENABLE_STATOR_CURRENT_LIMIT = true;
        // configuration.CURRENT_DEADBAND = 0.1;
        configuration.USE_FOC = true;
        logMetrics("Current Draw Good (amps)", "Current Draw Bad (amps)", "RPM Good Motor", "RPM Bad Motor");
        
        _goodTalon.configure(configuration);
        _badTalon.configure(configuration);
    }

    public void setSpeed(double speed) {
        _goodTalon.setPercentOutput(speed);
        _badTalon.setPercentOutput(speed);
    }

    @Override
    public void updateDashboard() {
        metric("Current Draw Good (amps)", _goodTalon.getSupplyCurrent().getValue());
        metric("Current Draw Bad (amps)", _badTalon.getSupplyCurrent().getValue());
        metric("RPM Good Motor", _goodTalon.getRotorVelocity().getValue() * 60);
        metric("RPM Bad Motor", _badTalon.getRotorVelocity().getValue() * 60);
    }
    
}