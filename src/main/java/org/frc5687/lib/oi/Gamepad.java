/* Team 5687 (C)2020-2022 */
package org.frc5687.lib.oi;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/** Copied by Caleb on 1/13/2017. */

/**
 * Handle input from standard Joysticks connected to the Driver Station. This class handles standard
 * input that comes from the Driver Station. Each time a value is requested the most recent value is
 * returned. There is a single class instance for each joystick and the mapping of ports to hardware
 * buttons depends on the code in the driver station.
 */
public class Gamepad extends Joystick {


    private final int _port;

    private final JoystickButton _a;
    private final JoystickButton _b;
    private final JoystickButton _x;
    private final JoystickButton _y;
    private final JoystickButton _leftBumper;
    private final JoystickButton _rightBumper;
    private final JoystickButton _back;
    private final JoystickButton _start;
    private final JoystickButton _leftStick;
    private final JoystickButton _rightStick;

    /** Enumeration for the various analog axes */
    public static enum Axes {
        LEFT_X(0),
        LEFT_Y(1),
        LEFT_TRIGGER(2),
        RIGHT_TRIGGER(3),
        RIGHT_X(4),
        RIGHT_Y(5),
        D_PAD_HORIZONTAL(6),
        D_PAD_VERTICAL(7);

        private final int number;

        Axes(int number) {
            this.number = number;
        }

        public int getNumber() {
            return number;
        }
    }

    /** Enumeration for the various buttons */
    public static enum Buttons {
        A(1),
        B(2),
        X(3),
        Y(4),
        LEFT_BUMPER(5),
        RIGHT_BUMPER(6),
        BACK(7),
        START(8),
        LEFT_STICK(9),
        RIGHT_STICK(10);

        private final int number;

        Buttons(int number) {
            this.number = number;
        }

        public int getNumber() {
            return number;
        }
    }

    /**
     * Constructor
     *
     * @param port the driver station port the gamepad is connected to
     */
    public Gamepad(int port) {
        super(port);
        _port = port;
        _a = new JoystickButton(this, Buttons.A.getNumber());
        _b = new JoystickButton(this, Buttons.B.getNumber());
        _x = new JoystickButton(this, Buttons.X.getNumber());
        _y = new JoystickButton(this, Buttons.Y.getNumber());

        _leftBumper = new JoystickButton(this, Buttons.LEFT_BUMPER.getNumber());
        _rightBumper = new JoystickButton(this, Buttons.RIGHT_BUMPER.getNumber());

        _back = new JoystickButton(this, Buttons.BACK.getNumber());
        _start = new JoystickButton(this, Buttons.START.getNumber());

        _leftStick = new JoystickButton(this, Buttons.LEFT_STICK.getNumber());
        _rightStick = new JoystickButton(this, Buttons.RIGHT_STICK.getNumber());
    }

    /**
     * Gets the raw value for the specified axis
     *
     * @param axis the desired gamepad axis
     * @return double the analog value for the axis
     */
    public double getRawAxis(Axes axis) {
        return super.getRawAxis(axis.getNumber());
    }

    public double getDirectionRadians(double xAxisValue, double yAxisValue) {
        return Math.atan2(-xAxisValue, -yAxisValue);
    }

    /**
     * Checks if the specified button is pressed
     *
     * @param button the desired gamepad button
     * @return bool true if the button is pressed
     */
    public boolean getRawButton(Buttons button) {
        return super.getRawButton(button.getNumber());
    }

    public JoystickButton getAButton() {
        return _a;
    }

    public JoystickButton getBButton() {
        return _b;
    }

    public JoystickButton getXButton() {
        return _x;
    }

    public JoystickButton getYButton() {
        return _y;
    }

    public JoystickButton getLeftBumper() {
        return _leftBumper;
    }

    public JoystickButton getRightBumper() {
        return _rightBumper;
    }

    public JoystickButton getBackButton() {
        return _back;
    }

    public JoystickButton getStartButton() {
        return _start;
    }

    public JoystickButton getLeftStickButton() {
        return _leftStick;
    }

    public JoystickButton getRightStickButton() {
        return _rightStick;
    }

    public boolean isAPressed() {
        return getRawButton(Buttons.A.getNumber());
    }

    public boolean isBPressed() {
        return getRawButton(Buttons.B.getNumber());
    }

    public boolean isXPressed() {
        return getRawButton(Buttons.X.getNumber());
    }

    public boolean isYPressed() {
        return getRawButton(Buttons.Y.getNumber());
    }

    public boolean isStartPressed() {
        return getRawButton(Buttons.START.getNumber());
    }

    public boolean isBackPressed() {
        return getRawButton(Buttons.BACK.getNumber());
    }

    public boolean isRightBumperPressed() {
        return getRawButton(Buttons.RIGHT_BUMPER.getNumber());
    }

    public boolean isLeftBumperPressed() {
        return getRawButton(Buttons.LEFT_BUMPER.getNumber());
    }

    public boolean isLeftStickPresssed() {
        return getRawButton(Buttons.LEFT_STICK.getNumber());
    }

    public boolean isRightStickPresssed() {
        return getRawButton(Buttons.RIGHT_STICK.getNumber());
    }


    /**
     * Get the button value (starting at button 1).
     *
     * <p>
     * The buttons are returned in a single 16 bit value with one bit representing
     * the state of
     * each button. The appropriate button is returned as a boolean value.
     *
     * <p>
     * This method returns true if the button is being held down at the time that
     * this method is
     * being called.
     *
     * @param button The button number to be read (starting at 1)
     * @return The state of the button.
     */
    public boolean getRawButton(int button) {
        return DriverStation.getStickButton(_port, (byte) button);
  }
}

