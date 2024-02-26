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
    public static final double METRIC_FLUSH_PERIOD = 0.02;
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

        public static final double WHEEL_RADIUS = Units.inchesToMeters(1.886); // new wheel tread measured by amory with calipers on 02/25/24
        public static final double GEAR_RATIO_DRIVE_LOW = (52.0 / 13.0) * (54.0 / 42.0) * (45.0 / 15.0) * (16.0 / 36.0); // 6.857142857142858
        public static final double GEAR_RATIO_DRIVE_HIGH = (52.0 / 13.0) * (44.0 / 52.0) * (45.0 / 15.0) * (16.0 / 36.0); // 4.512820512820512
        public static final double GEAR_RATIO_STEER = (52.0 / 14.0) * (96.0 / 16.0); // 22.2857

        public static final double IDLE_MPS_LIMIT = 0.05; // mps
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
            CONFIG.CURRENT_DEADBAND = 0.1;
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
            DRIVE_CONTROLLER_CONFIG.kP = 15.0;
            DRIVE_CONTROLLER_CONFIG.kI = 0.0;
            DRIVE_CONTROLLER_CONFIG.kD = 0.00;
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
            STEER_CONTROLLER_CONFIG.kP = 70;
            STEER_CONTROLLER_CONFIG.kI = 0;
            STEER_CONTROLLER_CONFIG.kD = 0.7;
            STEER_CONTROLLER_CONFIG.kF = 0.0;

            STEER_CONTROLLER_CONFIG.CRUISE_VELOCITY = 1000;
            STEER_CONTROLLER_CONFIG.ACCELERATION = 4000;
            STEER_CONTROLLER_CONFIG.JERK = 10000;

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
        public static final double MAX_LOW_GEAR_MPS = (
            Units.rotationsPerMinuteToRadiansPerSecond(MAX_KRAKEN_FOC_RPM) 
            / SwerveModule.GEAR_RATIO_DRIVE_LOW) * SwerveModule.WHEEL_RADIUS;
        public static final double MAX_HIGH_GEAR_MPS = (
            Units.rotationsPerMinuteToRadiansPerSecond(MAX_KRAKEN_FOC_RPM) 
            / SwerveModule.GEAR_RATIO_DRIVE_HIGH) * SwerveModule.WHEEL_RADIUS;

        public static final double OPTIMAL_SHIFT_MPS = 0.3 * MAX_HIGH_GEAR_MPS;

        public static final double SLOW_MPS = 2.0; // Slow speed of robot (m/s)
        public static final double MAX_ANG_VEL = 2.0 * Math.PI; // Max rotation rate of robot (rads/s)
        public static final double SLOW_ANG_VEL = Math.PI; // Max rotation rate of robot (rads/s)

        public static final double SHIFT_UP_SPEED_MPS = 2.5; // Speed to start shift y
        public static final double SHIFT_DOWN_SPEED_MPS = 1.5; // Speed to start shift y

        public static final double SHIFT_LOCKOUT = 250; // Time in milliseconds to wait before shifting again.

        public static final double MIN_TRANSLATION_COMMAND = 0.1; // mps
        public static final double YAW_RATE_THRESHOLD = 0.05; // rad / s

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

        public static final KinematicLimits VISION_KINEMATIC_LIMITS = new KinematicLimits();

        static {
            VISION_KINEMATIC_LIMITS.maxDriveVelocity = 1; // m/s
            VISION_KINEMATIC_LIMITS.maxDriveAcceleration = 5; // m/s^2
            VISION_KINEMATIC_LIMITS.maxSteeringVelocity = 10; // rad/s
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
            SOUTH_EAST_CONFIG.encoderOffset = -0.4846;
        }

        public static final ModuleConfiguration NORTH_EAST_CONFIG = new ModuleConfiguration();

        static {
            NORTH_EAST_CONFIG.moduleName = "North East";
            NORTH_EAST_CONFIG.canBus = CAN_BUS;
            NORTH_EAST_CONFIG.position = new Translation2d(SWERVE_NS_POS, -SWERVE_WE_POS); // +,-

            NORTH_EAST_CONFIG.encoderInverted = false;
            NORTH_EAST_CONFIG.encoderOffset = -0.2512;
        }

        public static final ModuleConfiguration NORTH_WEST_CONFIG = new ModuleConfiguration();

        static {
            NORTH_WEST_CONFIG.moduleName = "North West";
            NORTH_WEST_CONFIG.canBus = CAN_BUS;
            NORTH_WEST_CONFIG.position = new Translation2d(SWERVE_NS_POS, SWERVE_WE_POS); // +,+

            NORTH_WEST_CONFIG.encoderInverted = false;
            NORTH_WEST_CONFIG.encoderOffset = -0.3396;
        }

        public static final ModuleConfiguration SOUTH_WEST_CONFIG = new ModuleConfiguration();

        static {
            SOUTH_WEST_CONFIG.moduleName = "South West";
            SOUTH_WEST_CONFIG.canBus = CAN_BUS;
            SOUTH_WEST_CONFIG.position = new Translation2d(-SWERVE_NS_POS, SWERVE_WE_POS); // -,+

            SOUTH_WEST_CONFIG.encoderInverted = false;
            SOUTH_WEST_CONFIG.encoderOffset = -0.4974;
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

        public static final double SNAP_kP = 8.0;
        public static final double SNAP_kI = 0.0;
        public static final double SNAP_kD = 0.5;

        public static final double SNAP_TOLERANCE = Units.degreesToRadians(1.0);

        public static final double PROFILE_CONSTRAINT_VEL = Math.PI * 4.0;
        public static final double PROFILE_CONSTRAINT_ACCEL = Math.PI * 8.0;

        // AutoAlignDriveController PID
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
        public static final double HEADING_TOLERANCE = 0.02; // rad
        public static final double BUMP_DEGREES = 7;

        public static final double PITCH_LOOKING_ANGLE = Units.degreesToRadians(15.0); // this is degrees because sad.
        public static final double PITCH_LEVELED_ANGLE = Units.degreesToRadians(5.0); // this is degrees because sad.

        public static final double DRIVING_UP_RAMP_SPEEDS_VX = 2.0;
        public static final double DRIVING_DOWN_RAMP_SPEEDS_VX = 1.0;

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
                                                                                // more;

        public static double VISION_STD_DEV_X = 0.55;
        public static double VISION_STD_DEV_Y = 0.55;
        public static double VISION_STD_DEV_ANGLE = Units.degreesToRadians(900); // imu deviations lower number to trust
                                                                                // more;
    }

    public static class Shooter {
        public static final double VELOCITY_TOLERANCE = 30; // FIXME: inaccurate???

        public static final double IDLE_RPM = 500;

        public static final double DUNKER_IN_RPM = 700;
        public static final double DUNKER_OUT_RPM = 1000;

        public static final double OPTIMAL_SHOT_DISTANCE_THRESHOLD = 4.0;

        public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kHoodMap = new InterpolatingTreeMap<>();
        public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kRPMMap = new InterpolatingTreeMap<>();

        public static PolynomialRegression kRPMRegression;

        public static double[][] kRPMValues = {
            { 3.2, 2600 },
            { 3.65, 2200 },
            { 4.26, 1900 },
            { 4.86, 1780 },
        };

        public static final Pose2d RED_AMP_SHOT_POSE = new Pose2d(FieldConstants.FIELD_LENGTH - 1.82, FieldConstants.FIELD_WIDTH - 0.762002, new Rotation2d(-Math.PI/2)); // 1.82 meters from red alliance wall, ~0.75 meters from amp, facing amp
        
        public static final Pose2d BLUE_AMP_SHOT_POSE = new Pose2d(1.82, FieldConstants.FIELD_WIDTH - 0.762002, new Rotation2d(-Math.PI/2)); // 1.82 meters from blue alliance wall, ~0.75 meters from amp, facing amp
        
        public static final double AMP_SHOT_SPEED = 700;

        static {
            for (double[] pair : kRPMValues) {
                kRPMMap.put(new InterpolatingDouble(pair[0]), new InterpolatingDouble(pair[1]));
            }

            kRPMRegression = new PolynomialRegression(kRPMValues, 1);
        }

        public static final OutliersTalon.ClosedLoopConfiguration SHOOTER_CONTROLLER_CONFIG = new OutliersTalon.ClosedLoopConfiguration();

        static {
            SHOOTER_CONTROLLER_CONFIG.SLOT = 0;
            SHOOTER_CONTROLLER_CONFIG.kP = 0.46;
            SHOOTER_CONTROLLER_CONFIG.kI = 0;
            SHOOTER_CONTROLLER_CONFIG.kD = 0.001;
            SHOOTER_CONTROLLER_CONFIG.kF = 0.147;

            SHOOTER_CONTROLLER_CONFIG.IS_CONTINUOUS = false;
        }

        public static final OutliersTalon.Configuration CONFIG = new OutliersTalon.Configuration();

        static {
            CONFIG.TIME_OUT = 0.1;
            
            CONFIG.NEUTRAL_MODE = NeutralModeValue.Coast;
            CONFIG.INVERTED = InvertedValue.CounterClockwise_Positive;

            CONFIG.MAX_VOLTAGE = 12.0;

            CONFIG.MAX_SUPPLY_CURRENT = 60;
            CONFIG.ENABLE_SUPPLY_CURRENT_LIMIT = true;
            CONFIG.CURRENT_DEADBAND = 0.1;
            CONFIG.USE_FOC = true;
        }
    }

    public static class Intake {
        public static final String CAN_BUS = "CANivore";
        public static final double INTAKE_SPEED = 1.0;
        public static final double INDEX_SPEED = 0.3;
        public static final double HANDOFF_SPEED = 0.75;
        public static final OutliersTalon.Configuration CONFIG = new OutliersTalon.Configuration();
        // this is the motor config for the swerve motors
        static {
            CONFIG.TIME_OUT = 0.1;

            CONFIG.NEUTRAL_MODE = NeutralModeValue.Coast;
            CONFIG.INVERTED = InvertedValue.Clockwise_Positive;

            CONFIG.MAX_VOLTAGE = 12.0;

            CONFIG.MAX_SUPPLY_CURRENT = 30;
            CONFIG.MAX_STATOR_CURRENT = 30;
            CONFIG.MAX_CURRENT = 30;
            CONFIG.ENABLE_SUPPLY_CURRENT_LIMIT = true;
            CONFIG.CURRENT_DEADBAND = 0.1;
            CONFIG.USE_FOC = true;
        }
    }

    public static class Dunker {
        public static final String CAN_BUS = "CANivore";
        public static final OutliersTalon.Configuration CONFIG = new OutliersTalon.Configuration();

        public static final double DUNKER_GEAR_RATIO = (84.0/8.0); //8:84
        
        static {
            CONFIG.TIME_OUT = 0.1;
            
            CONFIG.NEUTRAL_MODE = NeutralModeValue.Brake;
            CONFIG.INVERTED = InvertedValue.Clockwise_Positive;

            CONFIG.MAX_VOLTAGE = 12.0;

            CONFIG.MAX_CURRENT = 60;
            CONFIG.MAX_SUPPLY_CURRENT = 60;
            CONFIG.ENABLE_SUPPLY_CURRENT_LIMIT = true;
            CONFIG.CURRENT_DEADBAND = 0.1;
            CONFIG.USE_FOC = true;
        }
        
        public static final ClosedLoopConfiguration CLOSED_LOOP_CONFIG = new OutliersTalon.ClosedLoopConfiguration();

        static {
            CLOSED_LOOP_CONFIG.SLOT = 0;
            CLOSED_LOOP_CONFIG.kP = 15;
            CLOSED_LOOP_CONFIG.kI = 0;
            CLOSED_LOOP_CONFIG.kD = 0.0001;
            CLOSED_LOOP_CONFIG.kF = 0;

            CLOSED_LOOP_CONFIG.CRUISE_VELOCITY = 100;
            CLOSED_LOOP_CONFIG.ACCELERATION = 1000;
            CLOSED_LOOP_CONFIG.JERK = 5000;

            CLOSED_LOOP_CONFIG.IS_CONTINUOUS = false;
        }

        public static final double ANGLE_SYNC_TOLERANCE = Units.degreesToRadians(1.0);

        public static final double PREP_ANGLE = 1.25;
        public static final double DUNK_ANGLE = 2.18;
        public static final double STOWED_ANGLE = 4.16;
        public static final double CLIMB_ANGLE = 1.05;
        public static final double ANGLE_TOLERANCE = 0.02;
        public static final long EJECT_TIME = 1000; // 1 second
    }
    
    public static class Climber {
        public static final String CAN_BUS = "CANivore";
        public static final OutliersTalon.Configuration CONFIG = new OutliersTalon.Configuration();

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

        public static double UPPER_LIMIT = -2.25;
        public static double LOWER_LIMIT = 0.0;

        // This represents where the climber assumes zero is when it starts, should be when hook touches lightbar
        public static double ZERO_VALUE = 0.0; 

        public static double PREP_METERS = -2.15;
        public static double CLIMB_METERS = LOWER_LIMIT; // 0.2

        public static double CLIMBER_TRANSLATION = .05;

        public static double CLIMBER_GEAR_RATIO = 25.0; // FIXME: actually 48 - freyja

        public static double WINCH_RADIUS = 0.035052; 

        public static double CLIMBER_TOLERANCE = .05;
        public static final OutliersTalon.ClosedLoopConfiguration CLOSED_LOOP_CONFIG = new OutliersTalon.ClosedLoopConfiguration();
        static {
            CLOSED_LOOP_CONFIG.SLOT = 0;
            CLOSED_LOOP_CONFIG.kP = 4;
            CLOSED_LOOP_CONFIG.kI = 0;
            CLOSED_LOOP_CONFIG.kD = 0;
            CLOSED_LOOP_CONFIG.kF = 0;

            CLOSED_LOOP_CONFIG.CRUISE_VELOCITY = 100;
            CLOSED_LOOP_CONFIG.ACCELERATION = 500;
            CLOSED_LOOP_CONFIG.JERK = 10;

            CLOSED_LOOP_CONFIG.IS_CONTINUOUS = false;
        }
    }

    public static class FieldConstants {
        public static final double FIELD_LENGTH = 16.54175;
        public static final double FIELD_WIDTH = 8.0137;
    }

    public static class CANdle {
        public static int NUM_LED = 38;
        public static double SPEED = 0.1;
        public static TwinklePercent TWINKLEPERCENT = TwinklePercent.Percent42;
        public static TwinkleOffPercent TWINKLEOFFPERCENT = TwinkleOffPercent.Percent42;

        public static int[] RED = {255, 0, 0};
        public static int[] ORANGE = {255, 20, 0};
        public static int[] YELLOW = {255, 65, 0};
        public static int[] GREEN = {0, 255, 0};
        public static int[] CYAN = {0, 255, 255};
        public static int[] BLUE = {0, 0, 255};
        public static int[] PURPLE = {128, 0, 128};
        public static int[] PINK = {255, 105, 18};
        public static int[] WHITE = {0, 0, 0};

        public static int[] RUFOUS = {128, 0, 0};
        public static int[] ORANGE_RED = {255, 69, 0};
        public static int[] MAROON = {128, 0, 0};
        public static int[] GOLD = {212, 175, 55};
        public static int[] PURPLER = {64, 0, 64};
        public static int[] LESS_GREEN = {0, 64, 0};
        public static int[] MINTISH = {100, 255, 100};
        public static int[] LEAF00 = {30, 175, 0}; //epic color frfr
        public static int[] MILO_BLUE = {1, 52, 133};
    }
}