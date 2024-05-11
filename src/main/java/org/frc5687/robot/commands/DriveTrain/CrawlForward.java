package org.frc5687.robot.commands.DriveTrain;

import org.frc5687.robot.Constants;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.DriveTrain;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class CrawlForward extends OutliersCommand {

    private DriveTrain _driveTrain;

    public CrawlForward(DriveTrain driveTrain) {
        _driveTrain = driveTrain;
    }

    @Override
    public void initialize() {
        _driveTrain.setRobotCentric();
    }

    @Override
    public void execute() {
        _driveTrain.setVelocity(new ChassisSpeeds(Constants.DriveTrain.CRAWL_MPS, 0, 0));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        _driveTrain.setFieldCentric();
    }
}
