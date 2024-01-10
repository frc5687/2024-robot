/* Team 5687 (C)2022 */
package org.frc5687.lib.control;

import org.frc5687.robot.Constants;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

// use 1323's Swerve heading controller
public class SwerveHeadingController {

    private HeadingState _headingState;
    private Rotation2d _targetHeading;
    private final ProfiledPIDController _PIDController;

    private long _disableTime;

    public SwerveHeadingController(double kDt) {
        _PIDController =
                new ProfiledPIDController(
                        Constants.DriveTrain.MAINTAIN_kP,
                        Constants.DriveTrain.MAINTAIN_kI,
                        Constants.DriveTrain.MAINTAIN_kD,
                        new TrapezoidProfile.Constraints(
                                Constants.DriveTrain.PROFILE_CONSTRAINT_VEL,
                                Constants.DriveTrain.PROFILE_CONSTRAINT_ACCEL),
                        kDt);
        _PIDController.enableContinuousInput(-Math.PI, Math.PI);
        _headingState = HeadingState.OFF;
        _targetHeading = new Rotation2d();
        _disableTime = System.currentTimeMillis();
    }

    public HeadingState getHeadingState() {
        return _headingState;
    }

    public void setState(HeadingState state) {
        _headingState = state;
    }

    public void disable() {
        setState(HeadingState.OFF);
    }

    public void temporaryDisable() {
        _disableTime = System.currentTimeMillis() + Constants.DriveTrain.DISABLE_TIME;
        setState(HeadingState.TEMPORARY_DISABLE);
    }

    public void setMaintainHeading(Rotation2d heading) {
        _targetHeading = heading;
        setState(HeadingState.MAINTAIN);
    }

    public void setVisionHeading(Rotation2d heading) {
        _targetHeading = heading;
        setState(HeadingState.VISION);
    }

    public void setSnapHeading(Rotation2d heading) {
        _targetHeading = heading;
        setState(HeadingState.SNAP);
    }

    public Rotation2d getTargetHeading() {
        return _targetHeading;
    }

    public void setTargetHeading(Rotation2d targetHeading) {
        _targetHeading = targetHeading;
    }

    public double getRotationCorrection(Rotation2d heading) {
        double power = 0;
        switch (_headingState) {
            case OFF:
                break;
            case TEMPORARY_DISABLE:
                _targetHeading = heading;
                if (System.currentTimeMillis() > _disableTime) {
                    setState(HeadingState.MAINTAIN);
                }
            case MAINTAIN:
                _PIDController.setPID(
                        Constants.DriveTrain.MAINTAIN_kP,
                        Constants.DriveTrain.MAINTAIN_kI,
                        Constants.DriveTrain.MAINTAIN_kD);
                power = _PIDController.calculate(heading.getRadians(), _targetHeading.getRadians());
                break;
            case SNAP:
                _PIDController.setPID(
                        Constants.DriveTrain.SNAP_kP,
                        Constants.DriveTrain.SNAP_kI,
                        Constants.DriveTrain.SNAP_kD);
                power = _PIDController.calculate(heading.getRadians(), _targetHeading.getRadians());
                break;
            default:
                break;
        }
        if (Math.abs(heading.getRadians() - _targetHeading.getRadians())
                < Units.degreesToRadians(1.0)) {
            power = 0.0;
        }
        return power;
    }

    public enum HeadingState {
        OFF(0),
        TEMPORARY_DISABLE(1),
        MAINTAIN(2),
        SNAP(3),
        VISION(4);

        private final int _value;

        HeadingState(int value) {
            _value = value;
        }

        public int getValue() {
            return _value;
        }
    }
}

