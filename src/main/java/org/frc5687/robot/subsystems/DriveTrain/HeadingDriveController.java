package org.frc5687.robot.subsystems.DriveTrain;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import org.frc5687.robot.Constants;
import edu.wpi.first.math.util.Units;

import java.util.Optional;

public class HeadingDriveController {

    public enum Mode {
        MAINTAIN_HEADING,
        SNAP_TO_HEADING
    }

    private ProfiledPIDController _profiledPIDController;
    private Optional<Rotation2d> _headingGoal = Optional.empty();
    private double _lastYawRate = 0;
    private boolean _isActive = false;
    private Mode _mode = Mode.MAINTAIN_HEADING;
    private Rotation2d _currentMaintainedHeading;


    public HeadingDriveController() {
        TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(
                Units.degreesToRadians(Constants.DriveTrain.MAX_ANG_VEL),
                Units.degreesToRadians(Constants.DriveTrain.MAX_ANG_VEL * 4)); // accel

        _profiledPIDController = new ProfiledPIDController(
                Constants.DriveTrain.MAINTAIN_kP,
                Constants.DriveTrain.MAINTAIN_kI,
                Constants.DriveTrain.MAINTAIN_kD,
                constraints);
        _profiledPIDController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public void setTargetHeading(Rotation2d targetHeading) {
        _headingGoal = Optional.of(targetHeading);
        _isActive = true;
        _mode = Mode.SNAP_TO_HEADING;
    }

    public void maintainCurrentHeading(Rotation2d currentHeading) {
        _currentMaintainedHeading = currentHeading;
        _isActive = true;
        _mode = Mode.MAINTAIN_HEADING;
    }

    public ChassisSpeeds update(ChassisSpeeds desired, Rotation2d currentHeading) {
        if (!_isActive) {
            return desired; // original speeds if the controller is not active
        }

        double rotationSpeed = 0;
        switch (_mode) {
            case MAINTAIN_HEADING:
                if (!_headingGoal.isPresent()) {
                    _headingGoal = Optional.of(_currentMaintainedHeading);
                }
                rotationSpeed = _profiledPIDController.calculate(currentHeading.getRadians(), _headingGoal.get().getRadians());
                break;
            case SNAP_TO_HEADING:
                if (_headingGoal.isPresent()) {
                    rotationSpeed = _profiledPIDController.calculate(currentHeading.getRadians(), _headingGoal.get().getRadians());
                }
                break;
        }

        return new ChassisSpeeds(desired.vxMetersPerSecond, desired.vyMetersPerSecond, rotationSpeed);
    }

    public void reset(Rotation2d currentHeading) {
        _profiledPIDController.reset(currentHeading.getRadians());
        _headingGoal = Optional.empty();
        _isActive = false;
        _mode = Mode.MAINTAIN_HEADING; // Reset to default mode
    }

    public boolean isActive() {
        return _isActive;
    }

    private boolean alwaysUpdateRotation() {
        return false;
    }
}
