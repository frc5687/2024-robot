package org.frc5687.robot.commands.DriveTrain;

import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.DriveTrain;

import edu.wpi.first.math.geometry.Rotation2d;

public class ZeroIMU extends OutliersCommand {

    private DriveTrain _driveTrain;

    public ZeroIMU(DriveTrain driveTrain) {
        _driveTrain = driveTrain;
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
    @Override
    public void end(boolean interrupted) {
        _driveTrain.zeroGyroscope();
        _driveTrain.goToHeading(_driveTrain.getHeading());
    }
}
