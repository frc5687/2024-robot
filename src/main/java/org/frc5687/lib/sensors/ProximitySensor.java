package org.frc5687.lib.sensors;

import edu.wpi.first.wpilibj.DigitalInput;

public class ProximitySensor extends DigitalInput {

    public ProximitySensor(int channel) {
        super(channel);
    }

    public boolean get() {
        return !super.get();
    }
}
