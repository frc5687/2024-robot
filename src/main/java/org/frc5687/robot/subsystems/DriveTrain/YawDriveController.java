package org.frc5687.robot.subsystems.DriveTrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import org.frc5687.lib.control.SwerveHeadingController;
import org.frc5687.lib.control.SwerveHeadingController.HeadingState;

public class YawDriveController {

    public enum Mode {
        MAINTAIN_HEADING,
        SNAP_TO_HEADING
    }

    private SwerveHeadingController _headingController;

    private boolean _isActive = false;
    private boolean _lockHeading = false;

    public YawDriveController() {
        _headingController = new SwerveHeadingController(0.02);
    }

    public void setTargetHeading(Rotation2d targetHeading) {
        _isActive = true;
        _headingController.setSnapHeading(targetHeading);
    }

    public void maintainCurrentHeading(Rotation2d currentHeading) {
        _headingController.setMaintainHeading(currentHeading);
        _isActive = true;
    }

    public ChassisSpeeds update(ChassisSpeeds desired, Rotation2d currentHeading) {
        if (!_isActive) {
            return desired; // original speeds if the controller is not active
        }

        if (desired.omegaRadiansPerSecond == 0 && _headingController.getHeadingState() != HeadingState.SNAP) {
            if (!_lockHeading) {
                _headingController.temporaryDisable();
            }
            _lockHeading = true;
        } else if (_headingController.getHeadingState() != HeadingState.SNAP) {
            _headingController.disable();
            _lockHeading = false;
        }

        double correction = _headingController.getRotationCorrection(currentHeading);

        return new ChassisSpeeds(desired.vxMetersPerSecond, desired.vyMetersPerSecond, correction);
    }

    public void reset(Rotation2d currentHeading) {
        _isActive = false;
    }

    public boolean isActive() {
        return _isActive;
    }
}
