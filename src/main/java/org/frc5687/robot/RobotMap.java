/* Team 5687 (C)2020-2021 */
package org.frc5687.robot;

public class RobotMap {

    /**
     * There should be an entry here for each CAN device, preferrably grouped by
     * device type and then
     * in numerical order. Note that for CAN, ids must be unique per device type,
     * but not across
     * types. Thus, you cannot have two SparkMax controllers with Id 0, but you can
     * have a SparkMax
     * with Id 0 and a TalonSRX with Id 0.
     */
    public static class CAN {

        public static class TALONFX {

            // Ordered in order of CAN connection in loop.

            public static final int SOUTH_EAST_ROTATION = 0;
            public static final int SOUTH_EAST_TRANSLATION = 1;
            public static final int NORTH_EAST_ROTATION = 2;
            public static final int NORTH_EAST_TRANSLATION = 3;
            public static final int NORTH_WEST_ROTATION = 4;
            public static final int NORTH_WEST_TRANSLATION = 5;
            public static final int SOUTH_WEST_ROTATION = 6;
            public static final int SOUTH_WEST_TRANSLATION = 7;
        }

        public static class CANCODER {
                public static final int ENCODER_SE = 0;
                public static final int ENCODER_NE = 1;
                public static final int ENCODER_NW = 2;
                public static final int ENCODER_SW = 3;
        }

        public static class PIGEON {
            public static final int PIGEON = 0;
        }

        public static class CANDLE {
            public static final int PORT = 13;
        }
    }

    /**
     * There should be an entry here for each PWM port, preferrably in numerical
     * order. Note that for
     * PWM only one device can connect to each port, so the numbers should be
     * unique.
     */
    public static class PWM {}

    /**
     * There should be an entry here for each PCM port, preferrably in numerical
     * order. Note that for
     * PCM only one device can connect to each port, so the numbers should be
     * unique.
     */
    public static class PCM {
        public static final int SHIFTER_HIGH = 15; // these values may be flip-flopped
        public static final int SHIFTER_LOW = 0;

        public static final int RATCHET_RAISE = 4;
        public static final int RATCHET_LOWER = 5;
    }

    /**
     * There should be an entry here for each PDP breaker, preferrably in numerical
     * order. Note that
     * only on device can be connected to each breaker, so the numbers should be
     * unique.
     */
    public static class PDP {}

    /**
     * There should be an entry here for each Analgo port, preferrably in numerical
     * order. Note that
     * for Analog only one device can connect to each port, so the numbers should be
     * unique.
     */
    public static class Analog {}


    /**
     * There should be an entry here for each DIO port, preferrably in numerical
     * order. Note that for
     * DIO only one device can connect to each port, so the numbers should be
     * unique.
     */
    public static class DIO {
        // IS THIS A 5687 REFERENCE?!?!?!?!?!?!?
        public static final int DUNKER_ABS_ENCODER = 5;
        public static final int DUNKER_PROXIMITY_SENSOR = 6;
        
        public static final int BOTTOM_DONUT_SENSOR = 8;
        public static final int TOP_DONUT_SENSOR = 7;
        
        public static final int LOWER_HALL = 9;

    }
}