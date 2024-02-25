package org.frc5687.robot.commands.DriveTrain;

import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.DriveTrain;
import org.frc5687.lib.control.SwerveHeadingController;

public class ZeroIMU extends OutliersCommand {

    private DriveTrain _driveTrain;

    public ZeroIMU(DriveTrain driveTrain) {
        _driveTrain = driveTrain;
    }

    @Override
    public void initialize() {
        _driveTrain.zeroGyroscope();
        _driveTrain.setHeadingControllerState(SwerveHeadingController.HeadingState.OFF);
        _driveTrain.setLockHeading(false);
    }

    @Override
    public void execute() {
        super.execute();
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }
    
    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}
