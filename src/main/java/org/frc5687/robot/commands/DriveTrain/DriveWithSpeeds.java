package org.frc5687.robot.commands.DriveTrain;

import org.frc5687.robot.Constants;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.DriveTrain;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class DriveWithSpeeds extends OutliersCommand{

    private DriveTrain _driveTrain;
    private double _vx;
    private double _vy;
    
    public DriveWithSpeeds(DriveTrain driveTrain, double vx, double vy) {
        _driveTrain = driveTrain;
        _vx = vx;
        _vy = vy;
    }

    @Override
    public void execute() {
        super.execute();    
        _driveTrain.setKinematicLimits(Constants.DriveTrain.KINEMATIC_LIMITS);
        _driveTrain.setVelocity(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                _vx, _vy, 0, _driveTrain.getHeading()
            )
        );
    }

    
}
