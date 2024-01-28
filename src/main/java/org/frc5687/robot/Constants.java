/* Team 5687 (C)2020-2022 */
package org.frc5687.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

import org.frc5687.lib.drivers.OutliersTalon;
import org.frc5687.lib.swerve.SwerveSetpointGenerator.KinematicLimits;
import org.frc5687.robot.subsystems.SwerveModule.ModuleConfiguration;

public class Constants {
    public static final int TICKS_PER_UPDATE = 1;
    public static final double METRIC_FLUSH_PERIOD = 0.02;
    public static final double UPDATE_PERIOD = 0.02; // 20 ms
    public static final double CONTROL_PERIOD = 0.02; // 10 ms
    public static final double DATA_PERIOD = 0.004; // 20 ms
    public static final double EPSILON = 1e-9;

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

        // Size of the robot chassis in meters
        public static final double WIDTH = 0.5461; // meters
        public static final double LENGTH = 0.5461; // meters
        // Distance of swerve modules from center of robot
        public static final double SWERVE_NS_POS = LENGTH / 2.0;
        public static final double SWERVE_WE_POS = WIDTH / 2.0;

        public static final double MAX_MPS = 6.5; // Max speed of robot (m/s)
        public static final double MAX_LOW_GEAR_MPS = 3.5;
        public static final double MAX_HIGH_GEAR_MPS = 6.5; // 6.85
        public static final double SLOW_MPS = 2.0; // Slow speed of robot (m/s)
        public static final double MAX_ANG_VEL = Math.PI; // Max rotation rate of robot (rads/s)
        public static final double SLOW_ANG_VEL = Math.PI; // Max rotation rate of robot (rads/s)

        public static final double SHIFT_UP_SPEED_MPS = 2.0; // Speed to start shift y
        public static final double SHIFT_DOWN_SPEED_MPS = 1.75; // Speed to start shift y

        public static final double SHIFT_LOCKOUT = 250; // Time in milliseconds to wait before shifting again.

        public static final KinematicLimits HIGH_KINEMATIC_LIMITS = new KinematicLimits();

        static {
            HIGH_KINEMATIC_LIMITS.maxDriveVelocity = MAX_HIGH_GEAR_MPS; // m/s
            HIGH_KINEMATIC_LIMITS.maxDriveAcceleration = 60; // m/s^2
            HIGH_KINEMATIC_LIMITS.maxSteeringVelocity = 25; // rad/s
        }
        public static final KinematicLimits LOW_KINEMATIC_LIMITS = new KinematicLimits();

        static {
            LOW_KINEMATIC_LIMITS.maxDriveVelocity = MAX_LOW_GEAR_MPS; // m/s
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
         * 1) Set the offset in code to 0
         * 2) Find your CANcoder on Phoenix Tuner X by its device ID
         * 3) Open the Plot window and check the Position box
         * 4) Turn the module back to its intended position
         * 5) Set the offset in code to the opposite of what Phoenix Tuner is reading
         * ex. -0.5 should be 0.5 in code
         * 6) Repeat with all []\
         * modules
         */

        public static final ModuleConfiguration NORTH_WEST_CONFIG = new ModuleConfiguration();

        static {
            NORTH_WEST_CONFIG.moduleName = "North West";
            NORTH_WEST_CONFIG.canBus = CAN_BUS;
            NORTH_WEST_CONFIG.position = new Translation2d(SWERVE_NS_POS, SWERVE_WE_POS); // +,+

            NORTH_WEST_CONFIG.encoderInverted = false;
            NORTH_WEST_CONFIG.encoderOffset = 0.219;
        }

        public static final ModuleConfiguration SOUTH_WEST_CONFIG = new ModuleConfiguration();

        static {
            SOUTH_WEST_CONFIG.moduleName = "South West";
            SOUTH_WEST_CONFIG.canBus = CAN_BUS;
            SOUTH_WEST_CONFIG.position = new Translation2d(-SWERVE_NS_POS, SWERVE_WE_POS); // -,+

            SOUTH_WEST_CONFIG.encoderInverted = false;
            SOUTH_WEST_CONFIG.encoderOffset = 0.34033203125;
        }

        public static final ModuleConfiguration SOUTH_EAST_CONFIG = new ModuleConfiguration();

        static {
            SOUTH_EAST_CONFIG.moduleName = "South East";
            SOUTH_EAST_CONFIG.canBus = CAN_BUS;
            SOUTH_EAST_CONFIG.position = new Translation2d(-SWERVE_NS_POS, -SWERVE_WE_POS); // -,-

            SOUTH_EAST_CONFIG.encoderInverted = false;
            SOUTH_EAST_CONFIG.encoderOffset = 0.515;
        }

        public static final ModuleConfiguration NORTH_EAST_CONFIG = new ModuleConfiguration();

        static {
            NORTH_EAST_CONFIG.moduleName = "North East";
            NORTH_EAST_CONFIG.canBus = CAN_BUS;
            NORTH_EAST_CONFIG.position = new Translation2d(SWERVE_NS_POS, -SWERVE_WE_POS); // +,-

            NORTH_EAST_CONFIG.encoderInverted = false;
            NORTH_EAST_CONFIG.encoderOffset = -0.215576171875;
        }

        public static final double TRANSLATION_DEADBAND = 0.05; // Avoid unintentional joystick movement
        public static final double ROTATION_DEADBAND = 0.05; // Avoid unintentional joystick movement
        public static final long DISABLE_TIME = 500; // ms

        public static final double LINEAR_VELOCITY_REFERENCE = 0.5;

        // Maximum rates of motion

        public static final double POLE_THRESHOLD = Units.degreesToRadians(5.0);

        // PID controller settings
        public static final double MAINTAIN_kP = 6.5;
        public static final double MAINTAIN_kI = 0.0;
        public static final double MAINTAIN_kD = 0.3;

        public static final double SNAP_kP = 4.0;
        public static final double SNAP_kI = 0.0;
        public static final double SNAP_kD = 0.1;

        public static final double SNAP_TOLERANCE = Units.degreesToRadians(5.0);

        public static final double PROFILE_CONSTRAINT_VEL = Math.PI * 4.0;
        public static final double PROFILE_CONSTRAINT_ACCEL = Math.PI * 8.0;

        public static final double kP = 3.3;
        public static final double kI = 0.0;
        public static final double kD = 0.05;

        public static final double X_TRAJECTORY_kP = 3.8;
        public static final double X_TRAJECTORY_kI = 0.0;
        public static final double X_TRAJECTORY_kD = 0.02;

        public static final double Y_TRAJECTORY_kP = 3.8;
        public static final double Y_TRAJECTORY_kI = 0.0;
        public static final double Y_TRAJECTORY_kD = 0.02;

        public static final double ANGLE_TRAJECTORY_kP = 3.2;
        public static final double ANGLE_TRAJECTORY_kI = 0.0;
        public static final double ANGLE_TRAJECTORY_kD = 0.05;

        public static final double POSITION_TOLERANCE = 0.01;
        public static final double LEVEL_TOLERANCE = 0.5;
        public static final double HEADING_TOLERANCE = 0.15; // rad
        public static final double BUMP_DEGREES = 7;

        public static final double PITCH_LOOKING_ANGLE = Units.degreesToRadians(15.0); // this is degrees because sad.
        public static final double PITCH_LEVELED_ANGLE = Units.degreesToRadians(5.0); // this is degrees because sad.

        public static final double DRIVING_UP_RAMP_SPEEDS_VX = 2.0;
        public static final double DRIVING_DOWN_RAMP_SPEEDS_VX = 1.0;

        public static final double AUTO_LEVEL_KP = 4.5; // PID controller for leveling
        public static final double AUTO_LEVEL_KI = 0.0;
        public static final double AUTO_LEVEL_KD = 1.0;

        public static final double QUICK_LEVEL_KP = 3.0; // PID controller for leveling
        public static final double QUICK_LEVEL_KI = 0.0;
        public static final double QUICK_LEVEL_KD = 0.5;

        public static final double MIN_PSI = 40.0;
        public static final double MAX_PSI = 100.0;
    }

    public static class SwerveModule {
        public static final String CAN_BUS = "CANivore";
        public static final int NUM_MODULES = 4;

        public static final double kDt = 0.005;
        public static final OutliersTalon.Configuration CONFIG = new OutliersTalon.Configuration();
        public static final OutliersTalon.Configuration STEER_CONFIG = new OutliersTalon.Configuration();

        public static final double WHEEL_RADIUS = 0.0492125;
        public static final double GEAR_RATIO_DRIVE_HIGH = 4.9;
        public static final double GEAR_RATIO_DRIVE_LOW = 9.6;
        public static final double GEAR_RATIO_STEER = (52 / 14) * (96 / 16);

        // public static
        final double MAX_SPEED = 0;

        public static final double kP = 5.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        // this is the motor config for the swerve motors
        static {
            CONFIG.TIME_OUT = 0.1;

            CONFIG.NEUTRAL_MODE = NeutralModeValue.Brake;
            CONFIG.INVERTED = InvertedValue.CounterClockwise_Positive;

            CONFIG.MAX_VOLTAGE = 12.0;

            CONFIG.MAX_STATOR_CURRENT = 120;
            CONFIG.MAX_CURRENT = 120;
            CONFIG.ENABLE_STATOR_CURRENT_LIMIT = true;
            CONFIG.CURRENT_DEADBAND = 0.1;
        }

        static {
            STEER_CONFIG.TIME_OUT = 0.1;

            STEER_CONFIG.NEUTRAL_MODE = NeutralModeValue.Brake;
            STEER_CONFIG.INVERTED = InvertedValue.CounterClockwise_Positive;

            STEER_CONFIG.MAX_VOLTAGE = 12.0;

            STEER_CONFIG.MAX_STATOR_CURRENT = 120;
            STEER_CONFIG.MAX_CURRENT = 120;
            STEER_CONFIG.ENABLE_STATOR_CURRENT_LIMIT = true;
            STEER_CONFIG.CURRENT_DEADBAND = 0.1;
        }

        public static final OutliersTalon.ClosedLoopConfiguration DRIVE_CONTROLLER_CONFIG = new OutliersTalon.ClosedLoopConfiguration();

        static {
            // DRIVE_CONTROLLER_CONFIG.SLOT = 0;

            // use these PID values when shifted down
            DRIVE_CONTROLLER_CONFIG.kP = 11.0;// 11.0 //23.0
            DRIVE_CONTROLLER_CONFIG.kI = 0.0;
            DRIVE_CONTROLLER_CONFIG.kD = 0.02;
            DRIVE_CONTROLLER_CONFIG.kF = 0.0;
            // use these PID values when shifted up
            DRIVE_CONTROLLER_CONFIG.kP1 = 45.0;
            DRIVE_CONTROLLER_CONFIG.kI1 = 0;
            DRIVE_CONTROLLER_CONFIG.kD1 = 0.0;

            DRIVE_CONTROLLER_CONFIG.kF1 = 0.0;

            DRIVE_CONTROLLER_CONFIG.CRUISE_VELOCITY = 1500;
            DRIVE_CONTROLLER_CONFIG.ACCELERATION = 750;
            DRIVE_CONTROLLER_CONFIG.JERK = 1000;
        }
        public static final OutliersTalon.ClosedLoopConfiguration STEER_CONTROLLER_CONFIG = new OutliersTalon.ClosedLoopConfiguration();

        static {
            STEER_CONTROLLER_CONFIG.SLOT = 0;
            STEER_CONTROLLER_CONFIG.kP = 70; // 70
            STEER_CONTROLLER_CONFIG.kI = 0;
            STEER_CONTROLLER_CONFIG.kD = 0.7; // 0.7
            STEER_CONTROLLER_CONFIG.kF = 0.0;

            STEER_CONTROLLER_CONFIG.CRUISE_VELOCITY = 1000;
            STEER_CONTROLLER_CONFIG.ACCELERATION = 4000;
            STEER_CONTROLLER_CONFIG.JERK = 10000;

            STEER_CONTROLLER_CONFIG.IS_CONTINUOUS = true;
        }

    }

    public static class Vision {
        public static final double VISION_kP = 3.0;
        public static final double VISION_kI = 0.0;
        public static final double VISION_kD = 0.2;
    }

    public static class VisionConfig {
        public static double STATE_STD_DEV_X = 0.01;
        public static double STATE_STD_DEV_Y = 0.01;
        public static double STATE_STD_DEV_ANGLE = Units.degreesToRadians(0.5); // imu deviations lower number to trust
                                                                                // more;

        public static double VISION_STD_DEV_X = 0.35;
        public static double VISION_STD_DEV_Y = 0.35;
        public static double VISION_STD_DEV_ANGLE = Units.degreesToRadians(70); // imu deviations lower number to trust
                                                                                // more;
    }

    public static class Shooter {
        public static final double SHOOT_RPM = 500;

        public static final double VELOCITY_TOLERANCE = 10;

        public static final double IDLE_RPM = 500;

        public static final OutliersTalon.ClosedLoopConfiguration SHOOTER_CONTROLLER_CONFIG = new OutliersTalon.ClosedLoopConfiguration();

        static {
            SHOOTER_CONTROLLER_CONFIG.SLOT = 0;
            SHOOTER_CONTROLLER_CONFIG.kP = 0.5;
            SHOOTER_CONTROLLER_CONFIG.kI = 0;
            SHOOTER_CONTROLLER_CONFIG.kD = 0.0001;
            SHOOTER_CONTROLLER_CONFIG.kF = 0.135;

            // THESE VALUES ARE GUESSES BASED ON LITERALLY NOTHING
            SHOOTER_CONTROLLER_CONFIG.CRUISE_VELOCITY = (int) (SHOOT_RPM / 60);
            SHOOTER_CONTROLLER_CONFIG.ACCELERATION = 4000;
            SHOOTER_CONTROLLER_CONFIG.JERK = 10000;

            SHOOTER_CONTROLLER_CONFIG.IS_CONTINUOUS = false;
        }

        public static final OutliersTalon.Configuration CONFIG = new OutliersTalon.Configuration();
        // this is the motor config for the swerve motors
        static {
            CONFIG.TIME_OUT = 0.1;

            CONFIG.NEUTRAL_MODE = NeutralModeValue.Coast;
            CONFIG.INVERTED = InvertedValue.CounterClockwise_Positive;

            CONFIG.MAX_VOLTAGE = 12.0;

            CONFIG.MAX_STATOR_CURRENT = 60;
            CONFIG.MAX_CURRENT = 60;
            CONFIG.ENABLE_STATOR_CURRENT_LIMIT = true;
            CONFIG.CURRENT_DEADBAND = 0.1;
            CONFIG.USE_FOC = true;
        }
    }

    public static class Intake {
        public static final String CAN_BUS = "CANivore";
        public static final double INTAKE_SPEED = 0.9;
        public static final OutliersTalon.Configuration CONFIG = new OutliersTalon.Configuration();
        // this is the motor config for the swerve motors
        static {
            CONFIG.TIME_OUT = 0.1;

            CONFIG.NEUTRAL_MODE = NeutralModeValue.Coast;
            CONFIG.INVERTED = InvertedValue.Clockwise_Positive;

            CONFIG.MAX_VOLTAGE = 12.0;

            CONFIG.MAX_STATOR_CURRENT = 60;
            CONFIG.MAX_CURRENT = 60;
            CONFIG.ENABLE_STATOR_CURRENT_LIMIT = true;
            CONFIG.CURRENT_DEADBAND = 0.1;
            CONFIG.USE_FOC = true;
        }
    }

    public static class Deflector {
        public static final String CAN_BUS = "CANivore";
        public static final OutliersTalon.Configuration CONFIG = new OutliersTalon.Configuration();

        public static final double GEAR_RATIO = 56.0; // 56:1

        public static final double ANGLE_TOLERANCE = 0.01;
        public static final double LOWER_HALL_ANGLE = 0.0;
        public static final double UPPER_HALL_ANGLE = 2.5;

        static {
            CONFIG.TIME_OUT = 0.1;

            CONFIG.NEUTRAL_MODE = NeutralModeValue.Brake;
            CONFIG.INVERTED = InvertedValue.CounterClockwise_Positive;

            CONFIG.MAX_VOLTAGE = 12.0;

            CONFIG.MAX_STATOR_CURRENT = 60;
            CONFIG.MAX_CURRENT = 60;
            CONFIG.ENABLE_STATOR_CURRENT_LIMIT = true;
            CONFIG.CURRENT_DEADBAND = 0.1;
            CONFIG.USE_FOC = true;
        }
        public static final OutliersTalon.ClosedLoopConfiguration CLOSED_LOOP_CONFIG = new OutliersTalon.ClosedLoopConfiguration();
        static {
            CLOSED_LOOP_CONFIG.SLOT = 0;
            CLOSED_LOOP_CONFIG.kP = 7;
            CLOSED_LOOP_CONFIG.kI = 0;
            CLOSED_LOOP_CONFIG.kD = 0;
            CLOSED_LOOP_CONFIG.kF = 0;

            CLOSED_LOOP_CONFIG.CRUISE_VELOCITY = 1000;
            CLOSED_LOOP_CONFIG.ACCELERATION = 500;
            CLOSED_LOOP_CONFIG.JERK = 10;

            CLOSED_LOOP_CONFIG.IS_CONTINUOUS = false;
        }
    }
}
