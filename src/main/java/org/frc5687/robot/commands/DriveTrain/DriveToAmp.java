package org.frc5687.robot.commands.DriveTrain;

import org.frc5687.robot.Constants;
import org.frc5687.robot.OI;
import org.frc5687.robot.RobotState;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.DriveTrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class DriveToAmp extends OutliersCommand {
    private final DriveTrain _driveTrain;
    private final OI _oi;
    private final RobotState _robotState = RobotState.getInstance();
    private Pose2d _goalPose;
    private PIDController _controller;

    public DriveToAmp(DriveTrain driveTrain, OI oi) {
        _driveTrain = driveTrain;
        _oi = oi;
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        _goalPose = _driveTrain.isRedAlliance() ? Constants.Shooter.RED_AMP_SHOT_POSE : Constants.Shooter.BLUE_AMP_SHOT_POSE;
        _controller = new PIDController(
            Constants.DriveTrain.POSE_kP, 
            0, 
            0
        );

        Rotation2d headingRot = _driveTrain.isRedAlliance() ? new Rotation2d(-Math.PI / 2) : new Rotation2d(Math.PI / 2);
        _driveTrain.goToHeading(headingRot);
    }

    @Override
    public void execute() {
        Pose2d currentPose = _robotState.getEstimatedPose();
        double error = _goalPose.getX() - currentPose.getX();
        double output = _controller.calculate(error);
        Rotation2d rotation = _driveTrain.isRedAlliance() ? _driveTrain.getHeading().plus(new Rotation2d(Math.PI)) : _driveTrain.getHeading();
        _driveTrain.setVelocity(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                output, 
                _oi.getDriveY() * Constants.DriveTrain.MAX_MPS / 2.0,
                _driveTrain.getRotationCorrection(),
                rotation
            )
        );
    }

    @Override
    public boolean isFinished() {
        // return Math.abs(_robotState.getEstimatedPose().getX() - _goalPose.getX()) < Constants.DriveTrain.POSITION_TOLERANCE;
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        new SnapTo(_driveTrain, new Rotation2d(0)).schedule();
    }

}
