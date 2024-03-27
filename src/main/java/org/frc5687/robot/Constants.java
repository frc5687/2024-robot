/* Team 5687 (C)2020-2022 */
package org.frc5687.robot;

import org.frc5687.lib.cheesystuff.InterpolatingDouble;
import org.frc5687.lib.cheesystuff.InterpolatingTreeMap;
import org.frc5687.lib.cheesystuff.PolynomialRegression;
import org.frc5687.lib.drivers.OutliersTalon;
import org.frc5687.lib.drivers.OutliersTalon.ClosedLoopConfiguration;
import org.frc5687.lib.swerve.SwerveSetpointGenerator.KinematicLimits;
import org.frc5687.robot.subsystems.SwerveModule.ModuleConfiguration;

import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class Constants {
    public static final int TICKS_PER_UPDATE = 1;
    public static final double METRIC_FLUSH_PERIOD = 5;
    public static final double UPDATE_PERIOD = 0.02; // 20 ms
    public static final double CONTROL_PERIOD = 0.02; // 10 ms
    public static final double DATA_PERIOD = 0.01; // 20 ms
    public static final double EPSILON = 1e-9;

    public static class SwerveModule {
        public static final String CAN_BUS = "CANivore";
        public static final int NUM_MODULES = 4;

        public static final double kDt = 0.005;
        public static final OutliersTalon.Configuration CONFIG = new OutliersTalon.Configuration();
        public static final OutliersTalon.Configuration STEER_CONFIG = new OutliersTalon.Configuration();

        public static final double WHEEL_RADIUS = Units.inchesToMeters(1.886); // new wheel tread measured by amory with
                                                                               // calipers on 02/25/24
        public static final double GEAR_RATIO_DRIVE_LOW = 9.6; //(52.0 / 13.0) * (54.0 / 42.0) * (45.0 / 15.0) * (16.0 / 36.0); // 6.857142857142858
        public static final double GEAR_RATIO_DRIVE_HIGH = 4.9;
        // (52.0 / 13.0) * (44.0 / 52.0) * (45.0 / 15.0)
                // * (16.0 / 36.0); // 4.512820512820512
        public static final double GEAR_RATIO_STEER = (52.0 / 14.0) * (96.0 / 16.0); // 22.2857

        public static final double IDLE_MPS_LIMIT = 0.005; // mps
        public static final double SHIFT_TIME_SECONDS = 0.1; // 100ms time to shift

        // this is the motor config for the swerve motors
        static {
            CONFIG.TIME_OUT = 0.1;

            CONFIG.NEUTRAL_MODE = NeutralModeValue.Brake;
            CONFIG.INVERTED = InvertedValue.CounterClockwise_Positive;

            CONFIG.MAX_VOLTAGE = 12.0;

            CONFIG.MAX_CURRENT = 80; // Max control requeset current
            CONFIG.MAX_SUPPLY_CURRENT = 30; // if using a foc control request these dont do anything, modify max_current
            CONFIG.MAX_STATOR_CURRENT = 120;

            CONFIG.ENABLE_SUPPLY_CURRENT_LIMIT = false;
            CONFIG.ENABLE_STATOR_CURRENT_LIMIT = false;
            CONFIG.CURRENT_DEADBAND = 0.5;
        }

        static {
            STEER_CONFIG.TIME_OUT = 0.1;

            STEER_CONFIG.NEUTRAL_MODE = NeutralModeValue.Brake;
            STEER_CONFIG.INVERTED = InvertedValue.CounterClockwise_Positive;

            STEER_CONFIG.MAX_VOLTAGE = 12.0;

            STEER_CONFIG.MAX_CURRENT = 30; // Max control request current
            STEER_CONFIG.MAX_SUPPLY_CURRENT = 30; // if using a foc control request these dont do anything, modify
                                                  // max_current
            STEER_CONFIG.MAX_STATOR_CURRENT = 120;
            STEER_CONFIG.ENABLE_SUPPLY_CURRENT_LIMIT = false;
            STEER_CONFIG.ENABLE_STATOR_CURRENT_LIMIT = false;
            STEER_CONFIG.CURRENT_DEADBAND = 0.1;
        }

        public static final OutliersTalon.ClosedLoopConfiguration DRIVE_CONTROLLER_CONFIG = new OutliersTalon.ClosedLoopConfiguration();

        static {
            // DRIVE_CONTROLLER_CONFIG.SLOT = 0;

            // use these PID values when shifted down
            DRIVE_CONTROLLER_CONFIG.kP = 8.0;
            DRIVE_CONTROLLER_CONFIG.kI = 0.0;
            DRIVE_CONTROLLER_CONFIG.kD = 0.3;
            DRIVE_CONTROLLER_CONFIG.kV = 0.55;
            DRIVE_CONTROLLER_CONFIG.kA = 0.0;
            // DRIVE_CONTROLLER_CONFIG.kS = 0.2;
            // use these PID values when shifted up
            DRIVE_CONTROLLER_CONFIG.kP1 = 50.0;
            DRIVE_CONTROLLER_CONFIG.kI1 = 0;
            DRIVE_CONTROLLER_CONFIG.kD1 = 0.0;
            DRIVE_CONTROLLER_CONFIG.kV1 = 0.0;
        }
        public static final OutliersTalon.ClosedLoopConfiguration STEER_CONTROLLER_CONFIG = new OutliersTalon.ClosedLoopConfiguration();

        static {
            STEER_CONTROLLER_CONFIG.SLOT = 0;
            STEER_CONTROLLER_CONFIG.kP = 70;
            STEER_CONTROLLER_CONFIG.kI = 0;
            STEER_CONTROLLER_CONFIG.kD = 0.7;
            STEER_CONTROLLER_CONFIG.kV = 0.0;

            STEER_CONTROLLER_CONFIG.IS_CONTINUOUS = true;
        }
    }

    /**
     * Coordinate System
     *
     * <p>
     * (X, Y): X is N or S, N is + Y is W or E, W is +
     *
     * <p>
     * NW (+,+) NE (+,-)
     *
     * <p>
     * SW (-,+) SE (-,-)
     *
     * <p>
     * We go counter-counter clockwise starting at NW of chassis:
     *
     * <p>
     * NW, SW, SE, NE
     *
     * <p>
     * Note: when robot is flipped over, this is clockwise.
     */
    public static class DriveTrain {
        public static final String CAN_BUS = "CANivore";
        public static final int NUM_MODULES = 4;

        // Size of the wheelbase in meters
        public static final double WIDTH = 0.5461; // meters
        public static final double LENGTH = 0.5461; // meters
        // Distance of swerve modules from center of robot
        public static final double SWERVE_NS_POS = LENGTH / 2.0;
        public static final double SWERVE_WE_POS = WIDTH / 2.0;

        public static final double MAX_FALCON_FOC_RPM = 6080.0;
        public static final double MAX_KRAKEN_FOC_RPM = 5800.0;
        public static final double MAX_MPS = 6.5; // Max speed of robot (m/s)
        public static final double MAX_LOW_GEAR_MPS = (Units.rotationsPerMinuteToRadiansPerSecond(MAX_FALCON_FOC_RPM)
                / SwerveModule.GEAR_RATIO_DRIVE_LOW) * SwerveModule.WHEEL_RADIUS; // 3.18 m/s
        public static final double MAX_HIGH_GEAR_MPS = (Units.rotationsPerMinuteToRadiansPerSecond(MAX_FALCON_FOC_RPM)
                / SwerveModule.GEAR_RATIO_DRIVE_HIGH) * SwerveModule.WHEEL_RADIUS;

        public static final double OPTIMAL_SHIFT_MPS = 0.3 * MAX_HIGH_GEAR_MPS;

        public static final double SLOW_MPS = 2.0; // Slow speed of robot (m/s)
        public static final double MAX_ANG_VEL = 2.0 * Math.PI; // Max rotation rate of robot (rads/s)
        public static final double MAX_ANG_ACC = 2.0 * Math.PI; // Max angular acceleration of robot (rads/s^2)
        public static final double SLOW_ANG_VEL = Math.PI; // Max rotation rate of robot (rads/s)

        public static final double SHIFT_UP_SPEED_MPS = 2.5; // Speed to start shift y
        public static final double SHIFT_DOWN_SPEED_MPS = 1.5; // Speed to start shift y

        public static final double SHIFT_LOCKOUT = 250; // Time in milliseconds to wait before shifting again.

        public static final double MIN_TRANSLATION_COMMAND = 0.1; // mps
        public static final double YAW_RATE_THRESHOLD = 0.05; // rad / s

        public static final KinematicLimits HIGH_KINEMATIC_LIMITS = new KinematicLimits();

        static {
            HIGH_KINEMATIC_LIMITS.maxDriveVelocity = 4.0; // m/s
            HIGH_KINEMATIC_LIMITS.maxDriveAcceleration = Double.MAX_VALUE; // m/s^2
            HIGH_KINEMATIC_LIMITS.maxSteeringVelocity = 25; // rad/s
        }
        public static final KinematicLimits LOW_KINEMATIC_LIMITS = new KinematicLimits();

        static {
            LOW_KINEMATIC_LIMITS.maxDriveVelocity = 3.5; // m/s
            LOW_KINEMATIC_LIMITS.maxDriveAcceleration = 35; // m/s^2
            LOW_KINEMATIC_LIMITS.maxSteeringVelocity = 25; // rad/s
        }
        
        public static final KinematicLimits KINEMATIC_LIMITS = LOW_KINEMATIC_LIMITS;

        public static final KinematicLimits DRIVE_POSE_KINEMATIC_LIMITS = new KinematicLimits();
        static {
            DRIVE_POSE_KINEMATIC_LIMITS.maxDriveVelocity = 2.5; // m/s
            DRIVE_POSE_KINEMATIC_LIMITS.maxDriveAcceleration = 20; // m/s^2
            DRIVE_POSE_KINEMATIC_LIMITS.maxSteeringVelocity = 20; // rad/s
        }

        public static final KinematicLimits TRAJECTORY_FOLLOWING = new KinematicLimits();
        static {
            TRAJECTORY_FOLLOWING.maxDriveVelocity = 5.0; // m/s
            TRAJECTORY_FOLLOWING.maxDriveAcceleration = 20; // m/s^2
            TRAJECTORY_FOLLOWING.maxSteeringVelocity = 20; // rad/s
        }
        public static final KinematicLimits SLOW_KINEMATIC_LIMITS = new KinematicLimits();

        static {
            SLOW_KINEMATIC_LIMITS.maxDriveVelocity = 2; // m/s
            SLOW_KINEMATIC_LIMITS.maxDriveAcceleration = 10; // m/s^2
            SLOW_KINEMATIC_LIMITS.maxSteeringVelocity = 10; // rad/s
        }

        /*
         * How to find offsets:
         * 
         * 1) Open Phoenix Tuner
         * 2) Zero CanCoder
         * 3) Config Tab
         * 4) Refresh
         * 5) Use "magnet offset" as offset in code
         */

        public static final ModuleConfiguration SOUTH_EAST_CONFIG = new ModuleConfiguration();

        static {
            SOUTH_EAST_CONFIG.moduleName = "South East";
            SOUTH_EAST_CONFIG.canBus = CAN_BUS;
            SOUTH_EAST_CONFIG.position = new Translation2d(-SWERVE_NS_POS, -SWERVE_WE_POS); // -,-

            SOUTH_EAST_CONFIG.encoderInverted = false;
            SOUTH_EAST_CONFIG.encoderOffset = 0.400634;
        }

        public static final ModuleConfiguration NORTH_EAST_CONFIG = new ModuleConfiguration();

        static {
            NORTH_EAST_CONFIG.moduleName = "North East";
            NORTH_EAST_CONFIG.canBus = CAN_BUS;
            NORTH_EAST_CONFIG.position = new Translation2d(SWERVE_NS_POS, -SWERVE_WE_POS); // +,-

            NORTH_EAST_CONFIG.encoderInverted = false;
            NORTH_EAST_CONFIG.encoderOffset = 0.166992;
        }

        public static final ModuleConfiguration NORTH_WEST_CONFIG = new ModuleConfiguration();

        static {
            NORTH_WEST_CONFIG.moduleName = "North West";
            NORTH_WEST_CONFIG.canBus = CAN_BUS;
            NORTH_WEST_CONFIG.position = new Translation2d(SWERVE_NS_POS, SWERVE_WE_POS); // +,+

            NORTH_WEST_CONFIG.encoderInverted = false;
            NORTH_WEST_CONFIG.encoderOffset = 0.397949;
        }

        public static final ModuleConfiguration SOUTH_WEST_CONFIG = new ModuleConfiguration();

        static {
            SOUTH_WEST_CONFIG.moduleName = "South West";
            SOUTH_WEST_CONFIG.canBus = CAN_BUS;
            SOUTH_WEST_CONFIG.position = new Translation2d(-SWERVE_NS_POS, SWERVE_WE_POS); // -,+

            SOUTH_WEST_CONFIG.encoderInverted = false;
            SOUTH_WEST_CONFIG.encoderOffset = 0.113037;
        }

        public static final double TRANSLATION_DEADBAND = 0.05; // Avoid unintentional joystick movement
        public static final double ROTATION_DEADBAND = 0.05; // Avoid unintentional joystick movement
        public static final long DISABLE_TIME = 500; // ms

        public static final double LINEAR_VELOCITY_REFERENCE = 0.5;

        // Maximum rates of motion
        public static final double POLE_THRESHOLD = Units.degreesToRadians(5.0);

        // PID controller settings
        public static final double HEADING_kP = 4.8;
        public static final double HEADING_kI = 0.0;
        public static final double HEADING_kD = 0.3;

        public static final double SNAP_TOLERANCE = Units.degreesToRadians(1.5);
        public static final double TARGET_TOLERANCE = Units.degreesToRadians(1);

        // AutoAlignDriveController PID
        public static final double kP = 3.3;
        public static final double kI = 0.0;
        public static final double kD = 0.05;

        public static final double POSITION_TOLERANCE = 0.01;
        public static final double LEVEL_TOLERANCE = 0.5;
        public static final double HEADING_TOLERANCE = 0.04; // rad
        public static final double BUMP_DEGREES = 7;

        public static final double MIN_PSI = 80.0;
        public static final double MAX_PSI = 120.0;
    }

    public static class Vision {
        public static final double VISION_kP = 3.0;
        public static final double VISION_kI = 0.0;
        public static final double VISION_kD = 0.2;
        public static final double AMBIGUITY_TOLERANCE = 0.4;
    }

    public static class VisionConfig {
        public static double STATE_STD_DEV_X = 0.01;
        public static double STATE_STD_DEV_Y = 0.01;
        public static double STATE_STD_DEV_ANGLE = Units.degreesToRadians(0.5); // imu deviations lower number to trust
                                                                                // more

        // we can't change the odometry stddev easily,,,, just change the vision stddev
        // --xavier bradford 02/25/24
        public static class Auto {
            public static double VISION_STD_DEV_X = 0.35;
            public static double VISION_STD_DEV_Y = 0.35;
            public static double VISION_STD_DEV_ANGLE = Units.degreesToRadians(900); // imu deviations lower number to
                                                                                     // trust
        }

        public static class Teleop {
            public static double VISION_STD_DEV_X = 0.15;
            public static double VISION_STD_DEV_Y = 0.15;
            public static double VISION_STD_DEV_ANGLE = Units.degreesToRadians(900); // imu deviations lower number to
                                                                                     // trust
        }
    }
}