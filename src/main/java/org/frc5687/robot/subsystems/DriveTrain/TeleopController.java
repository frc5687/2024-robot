package org.frc5687.robot.subsystems.DriveTrain;

import org.frc5687.lib.swerve.SwerveSetpointGenerator.KinematicLimits;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class TeleopController {
    private static final double DEADBAND = 0.05;

    private double _controllerX = 0.0;
    private double _controllerY = 0.0;
    private double _controllerOmega = 0.0;

    public TeleopController() {
    }

    public void acceptDriveInput(double x, double y, double omega) {
        _controllerX = MathUtil.applyDeadband(x, DEADBAND);
        _controllerY = MathUtil.applyDeadband(y, DEADBAND);
        _controllerOmega = MathUtil.applyDeadband(omega, DEADBAND);
    }

    public ChassisSpeeds update(Rotation2d currentHeading, KinematicLimits limits) {
        double xSpeed = _controllerX * limits.maxDriveVelocity;
        double ySpeed = _controllerY * limits.maxDriveVelocity;
        double omegaSpeed = _controllerOmega * limits.maxSteeringVelocity;

        return ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omegaSpeed, currentHeading);
    }
}
