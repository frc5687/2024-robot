/* Team 5687 (C)2022 */
package org.frc5687.lib.control;

import org.frc5687.robot.Constants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// rewritten (probably poorly) by xavier bradford 03/09/24
public class SwerveHeadingController {

    // state machine
    private HeadingState _headingState;
    private Rotation2d _targetHeading;
    private final PIDController _PIDController;

    // the timestamp at which the heading controller will enable again (after being temporarily disabled)
    private long _disableTime;

    public SwerveHeadingController(double kDt) {
        _PIDController = new PIDController(
            Constants.DriveTrain.HEADING_kP,
            Constants.DriveTrain.HEADING_kI,
            Constants.DriveTrain.HEADING_kD,
            kDt
        );
                
        _PIDController.enableContinuousInput(-Math.PI, Math.PI);
        _headingState = HeadingState.OFF;
        _targetHeading = new Rotation2d();
        _disableTime = System.currentTimeMillis();

        // SmartDashboard.putNumber("HeadingController/kP", Constants.DriveTrain.HEADING_kP);
        // SmartDashboard.putNumber("HeadingController/kI", Constants.DriveTrain.HEADING_kI);
        // SmartDashboard.putNumber("HeadingController/kD", Constants.DriveTrain.HEADING_kD);
    }


    public void setState(HeadingState state) {
        _headingState = state;
    }

    public void disable() {
        _headingState = HeadingState.OFF;
    }

    /**
     * Temporarily disable the heading controller. It will be reenabled after a small amount of time.
     * 
     * @see Constants.DriveTrain.DISABLE_TIME
     */
    public void temporaryDisable() {
        _disableTime = System.currentTimeMillis() + Constants.DriveTrain.DISABLE_TIME;
        _headingState = HeadingState.TEMPORARY_DISABLE;
    }

    /**
     * Sets the target heading of the heading controller and enables it.
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

    /**
     * Get the output of the PID controller given a certain input.
     * 
     * @param heading The current heading of the drivetrain. This could be a gyro value or a vision value or some combination of both.
     * @param measuredSpeed measured speed of the robot [vx, vy, omega]
     * @param maxSpeed The max speed of the robot 
     * @return The "power" that the heading controller outputs.
     */
    public double getRotationCorrection(Rotation2d heading) {
        double power = 0;
        switch (_headingState) {
            case TEMPORARY_DISABLE:
                _targetHeading = heading;
                if (System.currentTimeMillis() > _disableTime) {
                    _headingState = HeadingState.ON;
                }
                break;
            case ON:
                // _PIDController.setPID(
                //         SmartDashboard.getNumber("HeadingController/kP", Constants.DriveTrain.HEADING_kP),
                //         SmartDashboard.getNumber("HeadingController/kI", Constants.DriveTrain.HEADING_kI),
                //         SmartDashboard.getNumber("HeadingController/kD", Constants.DriveTrain.HEADING_kD));
                power = _PIDController.calculate(heading.getRadians(), _targetHeading.getRadians());
                break;
            default:
                break;
        }
    
        // double error = _targetHeading.minus(heading).getRadians();
        if (isAtTargetAngle(heading)) {
            power = 0;
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

