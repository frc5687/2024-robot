package org.frc5687.robot.commands.Shifter;

import org.frc5687.robot.Constants;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.DriveTrain;
import org.frc5687.robot.subsystems.Shifter;

public class AutoShift extends OutliersCommand {
    private final Shifter _shifter;
    private final DriveTrain _driveTrain;

    private boolean _shiftLockout = false;
    private long _shiftTime = 0;

    public AutoShift(Shifter shifter, DriveTrain driveTrain) {
        _shifter = shifter;
        _driveTrain = driveTrain;
    }

    @Override
    public void execute() {
        double speed = _driveTrain.getSpeed();
        if (speed > Constants.DriveTrain.SHIFT_UP_SPEED_MPS && _driveTrain.getDesiredSpeed() > Constants.DriveTrain.SHIFT_UP_SPEED_MPS) {
            if (!_shiftLockout) {
                _shiftLockout = true;
                _shiftTime = System.currentTimeMillis();
                _shifter.shiftUp();
            }
            if (_shiftTime + Constants.DriveTrain.SHIFT_LOCKOUT < System.currentTimeMillis()) {
                _shiftLockout = false;
            }
        }

        if (speed < Constants.DriveTrain.SHIFT_DOWN_SPEED_MPS) {
            if (!_shiftLockout) {
                _shiftTime = System.currentTimeMillis();
                _shiftLockout = true;
                _shifter.shiftDown();
            }
            if (_shiftTime + Constants.DriveTrain.SHIFT_LOCKOUT < System.currentTimeMillis()) {
                _shiftLockout = false;
            }
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
