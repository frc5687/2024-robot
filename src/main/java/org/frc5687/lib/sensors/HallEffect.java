/* Team 5687 (C)2021-2022 */
package org.frc5687.lib.sensors;

import edu.wpi.first.wpilibj.DigitalInput;

public class HallEffect extends DigitalInput {

    public HallEffect(int channel) {
        super(channel);
    }

    public boolean get() {
        return !super.get();
    }
}
