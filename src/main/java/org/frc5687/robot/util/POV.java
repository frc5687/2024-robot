/* (C)2020-2021 */
package org.frc5687.robot.util;
/**
 * Control type for the POV stick/DPAD, which is read as an angle by WPILib.
 *
 * <p>Credit goes to team3128
 * (https://github.com/Team3128/frc-java-3128/commit/a0eb1e3ba9932def8da7e4b958fdaad314318614)
 *
 * <p>It can be in the center position, or it can be in one of the four cardinal directions or one
 * of the four subcardinal directions.
 *
 * <pre>
 *      8
 *   1     7
 *
 * 2    0    6
 *
 *   3     5
 *      4
 *      </pre>
 *
 * @author Narwhal
 */
public class POV {
    int _indexOnJoystick;
    int _directionValue;

    public int getIndexOnJoystick() {
        return _indexOnJoystick;
    }

    public int getDirectionValue() {
        return _directionValue;
    }

    /**
     * Construct a POV control from the angle it should match and its index on the joystick.
     *
     * <pre>
     *      8
     *   1     7
     *
     * 2    0    6
     *
     *   3     5
     *      4
     * </pre>
     *
     * @param directionValue
     */
    public POV(int _indexOnJoystick, int directionValue) {
        super();

        if (directionValue < 0 || directionValue > 8) {
            throw new IllegalArgumentException("Direction value out of range");
        }

        this._indexOnJoystick = _indexOnJoystick;
        this._directionValue = directionValue;
    }

    /**
     * Creates a POV control from the value returned by Joystick.getPOV()
     *
     * @param angle
     * @return
     */
    public static POV fromWPILIbAngle(int _indexOnJoystick, int angle) {
        if (angle < 0) {
            return new POV(_indexOnJoystick, 0);
        }

        int value = 8 - (angle / 45);

        return new POV(_indexOnJoystick, value);
    }

    @Override
    public int hashCode() {
        return _indexOnJoystick * 100000 + _directionValue * 1000 + 19;
    }

    @Override
    public boolean equals(Object object) {
        if (object instanceof POV) {
            POV otherPOV = (POV) object;
            if (otherPOV._indexOnJoystick == _indexOnJoystick) {
                if (otherPOV._directionValue == _directionValue) {
                    return true;
                }
            }
        }

        return false;
    }
}
