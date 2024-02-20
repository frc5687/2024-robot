package org.frc5687.robot.commands.DriveTrain;

import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotState;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.DriveTrain;
import org.frc5687.robot.subsystems.DriveTrain.ControlState;

import edu.wpi.first.math.geometry.Pose2d;

public class DriveToAmp extends OutliersCommand {
    private final DriveTrain _driveTrain;
    private final RobotState _robotState = RobotState.getInstance();
    private boolean _isRed;

    public DriveToAmp(DriveTrain driveTrain) {
        _driveTrain = driveTrain;
        _isRed = false;
    }

    @Override
    public void initialize() {
        _driveTrain.setControlState(ControlState.POSITION);
        _isRed = _driveTrain.isRedAlliance();
        Pose2d goal =_isRed ? Constants.Shooter.RED_AMP_SHOT_POSE : Constants.Shooter.BLUE_AMP_SHOT_POSE;
        metric("Goal pose alliance", _isRed);
        new DriveToPose(_driveTrain, goal).schedule();
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

}
