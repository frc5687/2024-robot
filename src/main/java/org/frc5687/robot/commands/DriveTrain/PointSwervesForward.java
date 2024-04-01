package org.frc5687.robot.commands.DriveTrain;

import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.DriveTrain;

import edu.wpi.first.math.geometry.Rotation2d;

public class PointSwervesForward extends OutliersCommand {
    private DriveTrain _driveTrain;

    public PointSwervesForward(DriveTrain driveTrain) {
        _driveTrain = driveTrain;
        addRequirements(_driveTrain);
    }

    @Override
    public void execute() {
        // point all the modules forward
        _driveTrain.orientModules(new Rotation2d(0));
    }

    @Override
    public boolean isFinished() {
        return false; // use this with a whiletrue, it never ends
    }

    @Override
    public void end(boolean interrupted) {
    }
}
