/* Team 5687 (C)2020-2021 */
package org.frc5687.robot;

import static org.frc5687.robot.util.Helpers.applyDeadband;

import org.frc5687.lib.oi.AxisButton;
import org.frc5687.lib.oi.Gamepad;
import org.frc5687.robot.commands.DriveTrain.AutoAimSetpoint;
import org.frc5687.robot.commands.DriveTrain.DriveToAmp;
import org.frc5687.robot.commands.DriveTrain.DriveToNote;
import org.frc5687.robot.commands.DriveTrain.ShiftDown;
import org.frc5687.robot.commands.DriveTrain.SlowMode;
import org.frc5687.robot.commands.DriveTrain.SnapTo;
import org.frc5687.robot.commands.DriveTrain.ZeroIMU;
import org.frc5687.robot.commands.Dunker.DunkNote;
import org.frc5687.robot.commands.Dunker.HandoffDunker;
import org.frc5687.robot.commands.Intake.IntakeCommand;
import org.frc5687.robot.commands.Shooter.AutoShoot;
import org.frc5687.robot.commands.Shooter.ChangeRPM;
import org.frc5687.robot.commands.Shooter.IntakeEject;
import org.frc5687.robot.commands.Shooter.ManualShoot;
import org.frc5687.robot.commands.Shooter.Pass;
import org.frc5687.robot.commands.Shooter.ToggleAutoSpinUp;
import org.frc5687.robot.commands.Shooter.Shoot;
import org.frc5687.robot.commands.Shooter.ShooterEject;
import org.frc5687.robot.subsystems.Climber;
import org.frc5687.robot.subsystems.DriveTrain;
import org.frc5687.robot.subsystems.Dunker;
import org.frc5687.robot.subsystems.Intake;
import org.frc5687.robot.subsystems.Shooter;
import org.frc5687.robot.util.OutliersProxy;
import org.frc5687.robot.util.VisionProcessor;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class OI extends OutliersProxy {
    protected Gamepad _driverGamepad;
    protected Gamepad _operatorGamepad;
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
    protected Trigger _opPovButtonDown;
    protected Trigger _opPovButtonRight;
    protected Trigger _opPovButtonUp;
    protected Trigger _opPovButtonLeft;
    
    public OI() {

        _driverGamepad = new Gamepad(0);
        _operatorGamepad = new Gamepad(1);
        _buttonpad = new Gamepad(2);
        // _customController = new CustomController();
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
            Shooter shooter,
            Dunker dunker,
            Intake intake,
            Climber climber,
            VisionProcessor visionProcessor) {

        _driverLeftTrigger.whileTrue(new DriveToNote(drivetrain, intake).alongWith(new IntakeCommand(intake, this)));
        _driverRightTrigger.whileTrue(new Shoot(shooter, intake).alongWith(new AutoAimSetpoint(drivetrain)));

        // _driverGamepad.getAButton().onTrue(new AutoShoot(shooter, intake, drivetrain));

        _driverGamepad.getYButton().onTrue(new SnapTo(drivetrain, new Rotation2d(0)));
        _driverGamepad.getBButton().onTrue(new SnapTo(drivetrain, new Rotation2d(3 * Math.PI / 2)));
        _driverGamepad.getAButton().onTrue(new SnapTo(drivetrain, new Rotation2d(Math.PI)));
        _driverGamepad.getXButton().onTrue(new SnapTo(drivetrain, new Rotation2d(Math.PI / 2)));

        // _driverGamepad.getYButton().onTrue(new ChangeRPM(shooter, 100));
        // _driverGamepad.getAButton().onTrue(new ChangeRPM(shooter, -100));
        // _driverGamepad.getBButton().onTrue(new ChangeRPM(shooter, 10));
        // _driverGamepad.getXButton().onTrue(new ChangeRPM(shooter, -100));

        _driverGamepad.getRightBumper().whileTrue(new IntakeCommand(intake, this));
        
        _driverGamepad.getStartButton().onTrue(new ZeroIMU(drivetrain));
        // _povButtonLeft.onTrue(new AmpShot(shooter, deflector, drivetrain, intake));
        
        _driverGamepad.getLeftBumper().and(_driverGamepad.getRightBumper()).whileTrue(new DriveToAmp(drivetrain, this));
        //_driverGamepad.getLeftBumper().whileTrue(new ShiftDown(drivetrain));
        _driverGamepad.getLeftBumper().whileTrue(new SlowMode(drivetrain));

        _opPovButtonUp.onTrue(new ChangeRPM(shooter, 100));
        _opPovButtonDown.onTrue(new ChangeRPM(shooter, -100));
        _opPovButtonLeft.onTrue(new ChangeRPM(shooter, -10));
        _opPovButtonRight.onTrue(new ChangeRPM(shooter, 10));

        _operatorGamepad.getYButton().onTrue(new HandoffDunker(dunker, shooter, intake));
        _operatorGamepad.getXButton().onTrue(new DunkNote(dunker, shooter));
        _operatorGamepad.getBButton().whileTrue(new ShooterEject(shooter, intake));
        _operatorGamepad.getAButton().whileTrue(new IntakeEject(shooter, intake));

        _operatorGamepad.getLeftBumper().onTrue(new ToggleAutoSpinUp(shooter));
        _operatorGamepad.getRightBumper().whileTrue(new Pass(shooter, intake));


        _povButtonUp.whileTrue(new ManualShoot(shooter, intake));
    }

    public boolean shiftDown() {
        // return _driverGamepad.getLeftBumper().getAsBoolean();
        return false;
    }

    public boolean zeroIMU() {
        return _driverGamepad.getStartButton().getAsBoolean();
    }

    public boolean isShooting() {
        return _driverRightTrigger.getAsBoolean();
    }

    public boolean shiftUp() {
        // return _driverGamepad.getLeftBumper().getAsBoolean();
        return false;
    }

    public boolean getClimbButton() {
        return _operatorGamepad.getStartButton().getAsBoolean();
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

    public double getClimbY() {
        double speed = -getSpeedFromAxis(_operatorGamepad, Gamepad.Axes.LEFT_Y.getNumber());
        speed = applyDeadband(speed, Constants.Climber.CLIMBER_TRANSLATION);
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
