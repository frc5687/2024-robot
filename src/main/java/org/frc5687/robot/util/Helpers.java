/* Team 5687 (C)2020-2022 */
package org.frc5687.robot.util;

import edu.wpi.first.math.util.Units;

import org.frc5687.lib.math.Vector2d;


/** Created by Ben Bernard on 6/4/2018. */
public class Helpers {

    /** Limit motor values to the -1.0 to +1.0 range. */
    public static double limit(double value) {
        return limit(value, 1.0);
    }

    public static double limit(double value, double limitVal) {
        return limit(value, -limitVal, limitVal);
    }

    public static double limit(double value, double limitValLow, double limitValHigh) {
        if (value > limitValHigh) {
            return limitValHigh;
        }
        if (value < limitValLow) {
            return limitValLow;
        }
        return value;
    }

    /**
     * Returns 0.0 if the given value is within the specified range around zero. The remaining range
     * between the deadband and 1.0 is scaled from 0.0 to 1.0.
     *
     * @param value value to clip
     * @param deadband range around zero
     */
    public static double applyDeadband(double value, double deadband) {
        if (Math.abs(value) > deadband) {
            if (value > 0.0) {
                return (value - deadband) / (1.0 - deadband);
            } else {
                return (value + deadband) / (1.0 - deadband);
            }
        } else {
            return 0.0;
        }
    }

    /** Normalize all wheel speeds if the magnitude of any wheel is greater than 1.0. */
    public static void normalize(double[] wheelSpeeds) {
        double maxMagnitude = Math.abs(wheelSpeeds[0]);
        for (int i = 1; i < wheelSpeeds.length; i++) {
            double temp = Math.abs(wheelSpeeds[i]);
            if (maxMagnitude < temp) {
                maxMagnitude = temp;
            }
        }
        if (maxMagnitude > 1.0) {
            for (int i = 0; i < wheelSpeeds.length; i++) {
                wheelSpeeds[i] = wheelSpeeds[i] / maxMagnitude;
            }
        }
    }

    /**
     * Applies a transform to the input to provide better sensitivity at low speeds.
     *
     * @param input the raw input value from a joystick
     * @param factor the raw sensitivity factor to apply
     * @return the adjusted control value
     */
    public static double applySensitivityFactor(double input, double factor) {
        // See http://www.chiefdelphi.com/forums/showthread.php?p=921992

        // The transform can only work on values between -1 and 1.
        if (input > 1) {
            return 1;
        }
        if (input < -1) {
            return -1;
        }

        // The sensitivity factor MUST be between 0 and 1!
        double capped = Math.max(Math.min(factor, 1), 0);

        return capped * input * input * input + (1 - capped) * input;
    }

    /**
     * sets angle between -PI and PI.
     *
     * @param angle current to be changed.
     * @param radians determines if angle is radians or not.
     * @return changed angle.
     */
    public static double boundHalfAngle(double angle, boolean radians) {
        angle = radians ? angle : Units.degreesToRadians(angle);
        while (angle >= Math.PI) {
            angle -= 2.0 * Math.PI;
        }
        while (angle < -Math.PI) {
            angle += 2.0 * Math.PI;
        }
        return radians ? angle : Units.radiansToDegrees(angle);
    }

    /**
     * Wraps the angle so that its always between 0 -> (2 * pi)
     *
     * @param angle that needs to be wrapped
     * @param radians boolean to calculate radians vs degrees
     */
    public static double angleWrap(double angle, boolean radians) {
        angle = radians ? angle : Units.degreesToRadians(angle);
        angle = angle % (2.0 * Math.PI);
        if (angle < 0) {
            angle += (2.0 * Math.PI);
        }
        return radians ? angle : Units.radiansToDegrees(angle);
    }

    public static double joystickToAngle(double x, double y) {
        return Units.radiansToDegrees(Math.atan2(y, x));
    }

    public static Vector2d axisToSegmentedUnitCircleRadians(double x, double y, int[] array) {
        double angle = joystickToAngle(x, y);
        angle = (angle + 360) % 360;
        angle = angle / 360;
        int segment = (int) (angle * array.length);
        double power = Math.hypot(x, y);
        if (power > 1) {
            power = 1;
        }
        return new Vector2d(
                power * Math.cos(Units.degreesToRadians(array[segment])),
                power * Math.sin(Units.degreesToRadians(array[segment])));
    }

    public static boolean epsilonEquals(double a, double b, double epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }
}
