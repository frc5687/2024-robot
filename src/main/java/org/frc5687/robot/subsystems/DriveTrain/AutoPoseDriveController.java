package org.frc5687.robot.subsystems.DriveTrain;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import org.frc5687.robot.Constants;

public class AutoPoseDriveController {
    private ProfiledPIDController _xController;
    private ProfiledPIDController _yController;
    private ProfiledPIDController _headingController;

    private boolean _autoAlignComplete = false;
    private Pose2d _fieldToTargetPoint;
    private Double _startTime;

    public AutoPoseDriveController() {
        _xController = new ProfiledPIDController(
                Constants.DriveTrain.kP,
                Constants.DriveTrain.kI,
                Constants.DriveTrain.kD,
                new TrapezoidProfile.Constraints(
                        Constants.DriveTrain.DRIVE_POSE_KINEMATIC_LIMITS.maxDriveVelocity,
                        Constants.DriveTrain.DRIVE_POSE_KINEMATIC_LIMITS.maxDriveAcceleration));

        _yController = new ProfiledPIDController(
                Constants.DriveTrain.kP,
                Constants.DriveTrain.kI,
                Constants.DriveTrain.kD,
                new TrapezoidProfile.Constraints(
                        Constants.DriveTrain.DRIVE_POSE_KINEMATIC_LIMITS.maxDriveVelocity,
                        Constants.DriveTrain.DRIVE_POSE_KINEMATIC_LIMITS.maxDriveAcceleration));

        _headingController = new ProfiledPIDController(
                Constants.DriveTrain.MAINTAIN_kP,
                Constants.DriveTrain.MAINTAIN_kI,
                Constants.DriveTrain.MAINTAIN_kD,
                new TrapezoidProfile.Constraints(
                        Constants.DriveTrain.DRIVE_POSE_KINEMATIC_LIMITS.maxSteeringVelocity,
                        Constants.DriveTrain.DRIVE_POSE_KINEMATIC_LIMITS.maxSteeringVelocity * 4));
        _headingController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public void reset() {
        _startTime = Timer.getFPGATimestamp();
        _xController.reset(0, 0);
        _yController.reset(0, 0);
        _headingController.reset(0, 0);
        _autoAlignComplete = false;
    }

    public void setTargetPoint(Pose2d targetPoint) {
        _fieldToTargetPoint = targetPoint;
    }
    /***
     * Align(Drive) to target Pose2d
     * @param currentPose
     * @return ChassisSpeed to send to Swerve
     */

    public ChassisSpeeds updateAutoAlign(Pose2d currentPose) {
        Pose2d poseError = _fieldToTargetPoint.relativeTo(currentPose);

        double xOutput = _xController.calculate(currentPose.getX(), poseError.getX());
        double yOutput = _yController.calculate(currentPose.getY(), poseError.getY());
        double thetaOutput = _headingController.calculate(currentPose.getRotation().getRadians(), poseError.getRotation().getRadians());

        _autoAlignComplete = _xController.atGoal() && _yController.atGoal() && _headingController.atGoal();

        if (_autoAlignComplete && _startTime != null) {
            System.out.println("Auto align took: " + (Timer.getFPGATimestamp() - _startTime));
            _startTime = null;
        }

        return new ChassisSpeeds(xOutput, yOutput, thetaOutput);
    }

    public boolean isAutoAlignComplete() {
        return _autoAlignComplete;
    }
}
