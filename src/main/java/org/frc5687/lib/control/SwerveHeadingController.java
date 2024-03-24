/* Team 5687 (C)2022 */
package org.frc5687.lib.control;

import org.frc5687.robot.Constants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// rewritten (probably poorly) by xavier bradford 03/09/24
public class SwerveHeadingController {

    private HeadingState _headingState;
    private Rotation2d _targetHeading;
    private final PIDController _movingPIDController;
    private final PIDController _aimingPIDController;
    private double _velocityThresholdForFullAiming;
    private double _velocityThresholdForFullMoving;

    private long _disableTime;

    public SwerveHeadingController(double kDt) {
        _movingPIDController = new PIDController(
            SmartDashboard.getNumber("MovingHeadingController/kP", Constants.DriveTrain.MOVING_HEADING_kP),
            SmartDashboard.getNumber("MovingHeadingController/kI", Constants.DriveTrain.MOVING_HEADING_kI),
            SmartDashboard.getNumber("MovingHeadingController/kD", Constants.DriveTrain.MOVING_HEADING_kD),
            kDt
        );

        _aimingPIDController = new PIDController(
            SmartDashboard.getNumber("AimingHeadingController/kP", Constants.DriveTrain.AIMING_HEADING_kP),
            SmartDashboard.getNumber("AimingHeadingController/kI", Constants.DriveTrain.AIMING_HEADING_kI),
            SmartDashboard.getNumber("AimingHeadingController/kD", Constants.DriveTrain.AIMING_HEADING_kD),
            kDt
        );

        _movingPIDController.enableContinuousInput(-Math.PI, Math.PI);
        _aimingPIDController.enableContinuousInput(-Math.PI, Math.PI);
        _headingState = HeadingState.OFF;
        _targetHeading = new Rotation2d();
        _disableTime = System.currentTimeMillis();
        _velocityThresholdForFullAiming = SmartDashboard.getNumber("HeadingController/AimingThresholdMPS", 0.1);
        _velocityThresholdForFullMoving = SmartDashboard.getNumber("HeadingController/AimingThresholdMPS", 1.0);
    } 

    public void setState(HeadingState state) {
        _headingState = state;
    }

    public void disable() {
        _headingState = HeadingState.OFF;
    }

    /**
     * Temporarily disable the heading controller. It will be reenabled after a
     * small amount of time.
     * 
     * @see Constants.DriveTrain.DISABLE_TIME
     */
    public void temporaryDisable() {
        _disableTime = System.currentTimeMillis() + Constants.DriveTrain.DISABLE_TIME;
        _headingState = HeadingState.TEMPORARY_DISABLE;
    }

    /**
     * Sets the target heading of the heading controller and enables it.
     * 
     * @param targetHeading the heading to hold.
     */
    public void goToHeading(Rotation2d heading) {
        _targetHeading = heading;
        _headingState = HeadingState.ON;
    }

    public Rotation2d getTargetHeading() {
        return _targetHeading;
    }

    public boolean isAtTargetAngle(Rotation2d heading) {
        return (Math.abs(heading.minus(_targetHeading).getRadians()) < Constants.DriveTrain.HEADING_TOLERANCE);
    }

    public double getRotationCorrection(Rotation2d heading, ChassisSpeeds measuredSpeed) {
        double power = 0;

        double velocityMagnitude = Math.sqrt(measuredSpeed.vxMetersPerSecond * measuredSpeed.vxMetersPerSecond + measuredSpeed.vyMetersPerSecond * measuredSpeed.vyMetersPerSecond);
        double scale = Math.min(Math.max((velocityMagnitude - _velocityThresholdForFullAiming) / (_velocityThresholdForFullMoving - _velocityThresholdForFullAiming), 0), 1);

        _movingPIDController.setP(SmartDashboard.getNumber("MovingHeadingController/kP", Constants.DriveTrain.MOVING_HEADING_kP));
        _movingPIDController.setI(SmartDashboard.getNumber("MovingHeadingController/kI", Constants.DriveTrain.MOVING_HEADING_kI));
        _movingPIDController.setD(SmartDashboard.getNumber("MovingHeadingController/kD", Constants.DriveTrain.MOVING_HEADING_kD));

        _aimingPIDController.setP(SmartDashboard.getNumber("AimingHeadingController/kP", Constants.DriveTrain.AIMING_HEADING_kP));
        _aimingPIDController.setI(SmartDashboard.getNumber("AimingHeadingController/kI", Constants.DriveTrain.AIMING_HEADING_kI));
        _aimingPIDController.setD(SmartDashboard.getNumber("AimingHeadingController/kD", Constants.DriveTrain.AIMING_HEADING_kD));

        double movingPIDOutput = _movingPIDController.calculate(heading.getRadians(), _targetHeading.getRadians());
        double aimingPIDOutput = _aimingPIDController.calculate(heading.getRadians(), _targetHeading.getRadians());

        power = (1 - scale) * aimingPIDOutput + scale * movingPIDOutput;

        switch (_headingState) {
            case TEMPORARY_DISABLE:
                _targetHeading = heading;
                if (System.currentTimeMillis() > _disableTime) {
                    _headingState = HeadingState.ON;
                }
                break;
            case ON:
                if (isAtTargetAngle(heading)) {
                    power = 0;
                }
                break;
            default:
                break;
        }

        return power;
    }
   

    public enum HeadingState {
        OFF(0),
        TEMPORARY_DISABLE(1),
        ON(2);

        private final int _value;

        HeadingState(int value) {
            _value = value;
        }

        public int getValue() {
            return _value;
        }
    }
}
