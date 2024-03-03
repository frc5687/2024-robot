package org.frc5687.robot.commands.Shifter;

import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.Shifter;

public class HoldLowGear extends OutliersCommand {
    private final Shifter _shifter;

    /**
     * Only to be used with while true
     * @param shifter the Shifter subsystem
     */
    public HoldLowGear(Shifter shifter) {
        _shifter = shifter;
    }

    @Override
    public void initialize() {
        _shifter.shiftDown();
    }

    @Override
    public boolean isFinished() {
        // this does not end ON PURPOSE.
        // It is meant to be used as a while true. (ON PURPOSE) - xavier bradford 03/02/24
        return false;
    }
}
