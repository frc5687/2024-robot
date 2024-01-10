package org.frc5687.lib.control;

import org.frc5687.robot.Constants;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class HeadingController {
    public enum HeadingControllerState {
        OFF,
        SNAP,
        MAINTAIN,
    }

    private final ProfiledPIDController _PIDController;
    // private final PIDController _PIDController;

    private double _setpoint = 0.0;

    private final TrapezoidProfile.Constraints _constraints;
    private HeadingControllerState _headingControllerState = HeadingControllerState.OFF;

    public HeadingController(TrapezoidProfile.Constraints constraints) {
        _constraints = constraints;
        _PIDController =
                new ProfiledPIDController(
                        Constants.DriveTrain.MAINTAIN_kP,
                        Constants.DriveTrain.MAINTAIN_kI,
                        Constants.DriveTrain.MAINTAIN_kD,
                        _constraints);
        // _PIDController = new PIDController(
        //         Constants.DriveTrain.MAINTAIN_kP,
        //         Constants.DriveTrain.MAINTAIN_kI,
        //         Constants.DriveTrain.MAINTAIN_kD
        // );
        _PIDController.enableContinuousInput(-Math.PI, Math.PI);
        _PIDController.setTolerance(Constants.DriveTrain.HEADING_TOLERANCE);
    }

    public HeadingControllerState getHeadingControllerState() {
        return _headingControllerState;
    }

    public void setHeadingControllerState(HeadingControllerState state) {
        _headingControllerState = state;
    }

    /**
     * @param goal_pos pos in degrees
     */
    public void setGoal(double goalPos) {
        _setpoint = goalPos;
    }

    public double getGoal() {
        return _setpoint;
    }

    public boolean isAtGoal(double currentAngle) {
        return Math.abs(currentAngle - _setpoint) < Constants.DriveTrain.HEADING_TOLERANCE;
    }

    public void reset() {
        _PIDController.reset(_PIDController.getSetpoint());
    }

    public double calculate(double currentAngle) {
        _PIDController.setGoal(_setpoint);
        // _PIDController.setSetpoint(_setpoint);

        switch (_headingControllerState) {
            case OFF:
                return 0.0;
            case SNAP:
                _PIDController.setPID(
                        Constants.DriveTrain.SNAP_kP,
                        Constants.DriveTrain.SNAP_kI,
                        Constants.DriveTrain.SNAP_kD);
                break;
            case MAINTAIN:
                _PIDController.setPID(
                        Constants.DriveTrain.MAINTAIN_kP,
                        Constants.DriveTrain.MAINTAIN_kI,
                        Constants.DriveTrain.MAINTAIN_kD);
                break;
        }

        return _PIDController.calculate(currentAngle);
    }
}


