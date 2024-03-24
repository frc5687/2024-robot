/* Team 5687 (C)2020-2021 */
package org.frc5687.robot;

import static org.frc5687.robot.util.Helpers.applyDeadband;

import org.frc5687.lib.oi.AxisButton;
import org.frc5687.lib.oi.Gamepad;
import org.frc5687.robot.commands.DriveTrain.ShiftDown;
import org.frc5687.robot.commands.DriveTrain.ShiftUp;
import org.frc5687.robot.commands.DriveTrain.SnapTo;
import org.frc5687.robot.commands.DriveTrain.ZeroIMU;
import org.frc5687.robot.subsystems.DriveTrain;
import org.frc5687.robot.util.OutliersProxy;
import org.frc5687.robot.util.VisionProcessor;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class OI extends OutliersProxy {
    protected Gamepad _driverGamepad;
    protected Gamepad _operatorGamepad;
    protected Gamepad _buttonpad;
    
    protected Trigger _driverLeftTrigger;
    protected Trigger _driverRightTrigger;
    protected Trigger _buttonLeftTrigger;
    protected Trigger _buttonRightTrigger;
    protected Trigger _povButtonLeft;
    protected Trigger _povButtonRight;
    protected Trigger _povButtonUp;
    protected Trigger _povButtonDown;
    protected Trigger _opPovButtonDown;
    protected Trigger _opPovButtonRight;
    protected Trigger _opPovButtonUp;
    protected Trigger _opPovButtonLeft;
    
    public OI() {

        _driverGamepad = new Gamepad(0);
        _operatorGamepad = new Gamepad(1);
        _buttonpad = new Gamepad(2);
        _povButtonLeft = new Trigger(() -> _driverGamepad.getPOV() == 270);
        _povButtonRight = new Trigger(() -> _driverGamepad.getPOV() == 90);
        _povButtonUp = new Trigger(() -> _driverGamepad.getPOV() == 0);
        _povButtonDown = new Trigger(() -> _driverGamepad.getPOV() == 180);
        _opPovButtonDown = new Trigger(() -> _operatorGamepad.getPOV() == 180);
        _opPovButtonRight = new Trigger(() -> _operatorGamepad.getPOV() == 90);
        _opPovButtonUp = new Trigger(() -> _operatorGamepad.getPOV() == 0);
        _opPovButtonLeft = new Trigger(() -> _operatorGamepad.getPOV() == 270);

        _driverLeftTrigger = new Trigger(
                new AxisButton(_driverGamepad, Gamepad.Axes.LEFT_TRIGGER.getNumber(), 0.05)::get);
        _driverRightTrigger = new Trigger(
                new AxisButton(_driverGamepad, Gamepad.Axes.RIGHT_TRIGGER.getNumber(), 0.05)::get);
        _buttonLeftTrigger = new Trigger(new AxisButton(_buttonpad, Gamepad.Axes.LEFT_TRIGGER.getNumber(), 0.05)::get);
        _buttonRightTrigger = new Trigger(
                new AxisButton(_buttonpad, Gamepad.Axes.RIGHT_TRIGGER.getNumber(), 0.05)::get);

    }

    public void initializeButtons(
            DriveTrain drivetrain,
            VisionProcessor visionProcessor,
            RobotState robotState) {

        _driverGamepad.getYButton().onTrue(new SnapTo(drivetrain, new Rotation2d(0)));
        _driverGamepad.getBButton().onTrue(new SnapTo(drivetrain, new Rotation2d(3 * Math.PI / 2)));
        _driverGamepad.getAButton().onTrue(new SnapTo(drivetrain, new Rotation2d(Math.PI)));
        _driverGamepad.getXButton().onTrue(new SnapTo(drivetrain, new Rotation2d(3 * Math.PI / 2)));
        
        // _driverGamepad.getLeftBumper().onTrue(new ShiftDown(drivetrain));
        // _driverGamepad.getRightBumper().onTrue(new ShiftUp(drivetrain));

        _driverGamepad.getStartButton().onTrue(new ZeroIMU(drivetrain));
    }

    

    public boolean zeroIMU() {
        return _driverGamepad.getStartButton().getAsBoolean();
    }

    public boolean isShooting() {
        return _driverRightTrigger.getAsBoolean();
    }


    public double getDriveY() {
        double speed = -getSpeedFromAxis(_driverGamepad, Gamepad.Axes.LEFT_Y.getNumber());
        speed = applyDeadband(speed, Constants.DriveTrain.TRANSLATION_DEADBAND);
        return speed;
    }

    public double getDriveX() {
        double speed = -getSpeedFromAxis(_driverGamepad, Gamepad.Axes.LEFT_X.getNumber());
        speed = applyDeadband(speed, Constants.DriveTrain.TRANSLATION_DEADBAND);
        return speed;
    }
 
    public double getRotationX() {
        double speed = -getSpeedFromAxis(_driverGamepad, Gamepad.Axes.RIGHT_X.getNumber());
        speed = applyDeadband(speed, Constants.DriveTrain.ROTATION_DEADBAND);
        return speed;
    }

    protected double getSpeedFromAxis(Joystick gamepad, int axisNumber) {
        return gamepad.getRawAxis(axisNumber);
    }

    @Override
    public void updateDashboard() {
        // metric("Raw x", xIn);
        // metric("Raw y", yIn);
    }

    public void rumbleDriver() {
        _driverGamepad.setRumble(GenericHID.RumbleType.kBothRumble, 1);
    }

    public void stopRumbleDriver() {
        _driverGamepad.setRumble(GenericHID.RumbleType.kBothRumble, 0);

    }
}
