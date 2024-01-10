/* Team 5687 (C)2021-2022 */
package org.frc5687.lib.oi;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Created by Ben Bernard on 3/2/2017. */
public class AxisButton {
    private final GenericHID m_joystick;
    private final int m_axisNumber;
    private double _threshold;

    public AxisButton(GenericHID joystick, int axisNumber, double threshold) {
        this.m_joystick = joystick;
        this.m_axisNumber = axisNumber;
        _threshold = threshold;
    }

    public boolean get() {
        SmartDashboard.putNumber(
                "OI/Axis/" + m_axisNumber, this.m_joystick.getRawAxis(this.m_axisNumber));
        if (_threshold > 0) {
            return this.m_joystick.getRawAxis(this.m_axisNumber) > _threshold;
        } else return this.m_joystick.getRawAxis(this.m_axisNumber) < _threshold;
    }
}
