/* Team 5687 (C)2022 */
package org.frc5687.lib.vision;

import java.util.HashMap;
import java.util.Map;

public class TrackedObjectInfo {

    private final GameElement element;
    private final float x;
    private final float y;
    private final float z;

    private final float vx;
    private final float vy;
    private final float vz;

    public TrackedObjectInfo(
            GameElement element, float x, float y, float z, float vx, float vy, float vz) {
        this.element = element;
        this.x = x;
        this.y = y;
        this.z = z;
        this.vx = vx;
        this.vy = vy;
        this.vz = vz;
    }

    public GameElement getElement() {
        return element;
    }

    public float getX() {
        return x;
    }

    public float getY() {
        return y;
    }

    public float getZ() {
        return z;
    }

    public float getVX() {
        return vx;
    }

    public float getVY() {
        return vy;
    }

    public double getDistance() {
        return Math.sqrt(x * x + y * y + z * z);
    }

    public double getAzimuthAngle() {
        return Math.atan2(y, x);
    }

    public double getAltitudeAngle() {
        return Math.asin(z / getDistance());
    }

    public float getVZ() {
        return vz;
    }

    public static int size() {
        // 8 for uint8_t
        return 8 + (Float.SIZE * 6);
    }

    public static int sizeBytes() {
        return 1 + (Float.BYTES * 6);
    }

    public static int numberOfElements() {
        return 7;
    }

    public String toString() {
        return "Game Element: "
                + getElement().name()
                + ", x: "
                + getX()
                + ", y: "
                + getY()
                + ", z: "
                + getZ();
    }

    public enum GameElement {
        CONE(0),
        CUBE(1);

        private final int _value;
        private static final Map<Object, Object> map = new HashMap<>();

        GameElement(int value) {
            _value = value;
        }

        static {
            for (GameElement gameElement : GameElement.values()) {
                map.put(gameElement._value, gameElement);
            }
        }

        public static GameElement valueOf(int gameElement) {
            return (GameElement) map.get(gameElement);
        }

        public int getValue() {
            return _value;
        }
    }
}

