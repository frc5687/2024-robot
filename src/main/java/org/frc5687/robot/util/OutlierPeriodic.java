/* Team 5687 (C)2022 */
package org.frc5687.robot.util;

public interface OutlierPeriodic {
    /** Control processing periodic function */
    void controlPeriodic(double timestamp);

    /** Data processing periodic function */
    void dataPeriodic(double timestamp);
}
