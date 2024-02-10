/* Team 5687 (C)2020-2021 */
package org.frc5687.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static org.frc5687.robot.util.Helpers.*;

import org.frc5687.lib.oi.AxisButton;
import org.frc5687.lib.oi.Gamepad;
import org.frc5687.robot.commands.*;
import org.frc5687.robot.commands.Shooter.ChangeRPM;
import org.frc5687.robot.commands.Deflector.ChangeDeflectorAngle;
import org.frc5687.robot.commands.Intake.IntakeCommand;
import org.frc5687.robot.commands.Intake.IndexNote;
import org.frc5687.robot.commands.Shooter.Shoot;
import org.frc5687.robot.subsystems.*;
import org.frc5687.robot.util.OutliersProxy;

public class OI extends OutliersProxy {
    protected Gamepad _driverGamepad;
    protected CommandJoystick _operatorJoystick;
    protected Gamepad _buttonpad;

    // protected CustomController _customController;
    protected Trigger _driverLeftTrigger;
    protected Trigger _driverRightTrigger;
    protected Trigger _buttonLeftTrigger;
    protected Trigger _buttonRightTrigger;
    protected Trigger _povButtonLeft;
    protected Trigger _povButtonRight;
    protected Trigger _povButtonUp;
    protected Trigger _povButtonDown;

    public OI() {

        _driverGamepad = new Gamepad(0);
        _operatorJoystick = new CommandJoystick(1);
        _buttonpad = new Gamepad(2);
        // _customController = new CustomController();
        _povButtonLeft = new Trigger(() -> _driverGamepad.getPOV() == 270);
        _povButtonRight = new Trigger(() -> _driverGamepad.getPOV() == 90);
        _povButtonUp = new Trigger(() -> _driverGamepad.getPOV() == 0);
        _povButtonDown = new Trigger(() -> _driverGamepad.getPOV() == 180);
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
            Shooter shooter,
            Intake intake,
            Deflector deflector) {
        _driverLeftTrigger.whileTrue(new IntakeCommand(intake, this));
        _driverRightTrigger.whileTrue(new Shoot(shooter, deflector, intake, drivetrain));

        _driverGamepad.getYButton().onTrue(new SnapTo(drivetrain, new Rotation2d(0)));
        _driverGamepad.getBButton().onTrue(new SnapTo(drivetrain, new Rotation2d(Math.PI/2)));
        _driverGamepad.getAButton().onTrue(new SnapTo(drivetrain, new Rotation2d(Math.PI)));
        _driverGamepad.getXButton().onTrue(new SnapTo(drivetrain, new Rotation2d(3*Math.PI/2)));

        
        
        _povButtonUp.onTrue(new ChangeRPM(shooter, 100));
        _povButtonDown.onTrue(new ChangeRPM(shooter, -100));
        _povButtonRight.onTrue(new ChangeRPM(shooter, 10));
        _povButtonLeft.onTrue(new ChangeRPM(shooter, -10));

        _driverGamepad.getLeftBumper().onTrue(new ChangeDeflectorAngle(deflector, -0.05));
        // _driverGamepad.getRightBumper().onTrue(new ChangeDeflectorAngle(deflector, 0.05));
        _driverGamepad.getRightBumper().whileTrue(new DriveToPose(drivetrain, new Pose2d(2, 2, new Rotation2d())));
    }

    public boolean shiftUp() {
        // return _driverGamepad.getAButton().getAsBoolean();
        // return _driverGamepad.getBButton().getAsBoolean();
        return false;
    }

    public boolean shiftDown() {
        // return _driverGamepad.getBButton().getAsBoolean();
        // return _driverGamepad.getXButton().getAsBoolean(); //changed as vision no
        // worky rn
        return false;
    }

    public boolean shiftOverride() {
        return _driverGamepad.getBackButton().getAsBoolean();
    }

    public boolean getSlowMode() {
        return _driverGamepad.getLeftBumper().getAsBoolean();
    }

    public boolean zeroIMU() {
        return _driverGamepad.getStartButton().getAsBoolean();
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
