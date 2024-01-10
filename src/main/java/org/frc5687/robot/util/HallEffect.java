/* (C)2021 */
package org.frc5687.robot.util;

import edu.wpi.first.wpilibj.DigitalInput;

public class HallEffect extends DigitalInput {

    public HallEffect(int channel) {
        super(channel);
    }

    public boolean get() {
        return !super.get();
    }
}
