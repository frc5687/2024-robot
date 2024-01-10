package org.frc5687.robot.util;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import java.util.List;
public final class FieldConstants {
    public static final boolean isRealField = true; // Red alliance
    public static final double fieldLength = 16.54175;
    public static final double fieldWidth = 8.0137;

    // AprilTag constants
    public static final AprilTagFieldLayout aprilTags =
            isRealField
                    ? new AprilTagFieldLayout(
                    List.of(
                            new AprilTag(
                                    1,
                                    new Pose3d(
                                            Units.inchesToMeters(610.77),
                                            Units.inchesToMeters(42.19),
                                            Units.inchesToMeters(18.22),
                                            new Rotation3d(0.0, 0.0, Math.PI))),
                            new AprilTag(
                                    2,
                                    new Pose3d(
                                            Units.inchesToMeters(610.77),
                                            Units.inchesToMeters(108.19),
                                            Units.inchesToMeters(18.22),
                                            new Rotation3d(0.0, 0.0, Math.PI))),
                            new AprilTag(
                                    3,
                                    new Pose3d(
                                            Units.inchesToMeters(610.77),
                                            Units.inchesToMeters(174.19),
                                            Units.inchesToMeters(18.22),
                                            new Rotation3d(0.0, 0.0, Math.PI))),
                            new AprilTag(
                                    4,
                                    new Pose3d(
                                            Units.inchesToMeters(636.96),
                                            Units.inchesToMeters(265.74),
                                            Units.inchesToMeters(27.38),
                                            new Rotation3d(0.0, 0.0, Math.PI))),
                            new AprilTag(
                                    5,
                                    new Pose3d(
                                            Units.inchesToMeters(14.25),
                                            Units.inchesToMeters(265.74),
                                            Units.inchesToMeters(27.38),
                                            new Rotation3d())),
                            new AprilTag(
                                    6,
                                    new Pose3d(
                                            Units.inchesToMeters(40.45),
                                            Units.inchesToMeters(174.19),
                                            Units.inchesToMeters(18.22),
                                            new Rotation3d())),
                            new AprilTag(
                                    7,
                                    new Pose3d(
                                            Units.inchesToMeters(40.45),
                                            Units.inchesToMeters(108.19),
                                            Units.inchesToMeters(18.22),
                                            new Rotation3d())),
                            new AprilTag(
                                    8,
                                    new Pose3d(
                                            Units.inchesToMeters(40.45),
                                            Units.inchesToMeters(42.19),
                                            Units.inchesToMeters(18.22),
                                            new Rotation3d()))),
                    fieldLength,
                    fieldWidth)
                    : new AprilTagFieldLayout(
                    List.of(
                            new AprilTag(
                                    1,
                                    new Pose3d(
                                            fieldLength - 1.02,
                                            1.15,
                                            Units.inchesToMeters(18.22),
                                            new Rotation3d(0.0, 0.0, Math.PI))),
                            new AprilTag(
                                    2,
                                    new Pose3d(
                                            fieldLength - 1.02,
                                            2.84,
                                            Units.inchesToMeters(18.22),
                                            new Rotation3d(0.0, 0.0, Math.PI))),
                            new AprilTag(
                                    3,
                                    new Pose3d(
                                            fieldLength - 1.02,
                                            4.53,
                                            Units.inchesToMeters(18.22),
                                            new Rotation3d(0.0, 0.0, Math.PI))),
                            new AprilTag(
                                    4,
                                    new Pose3d(
                                            Units.inchesToMeters(636.96),
                                            Units.inchesToMeters(265.74),
                                            Units.inchesToMeters(27.38),
                                            new Rotation3d(0.0, 0.0, Math.PI))),
                            new AprilTag(
                                    5,
                                    new Pose3d(
                                            Units.inchesToMeters(14.25),
                                            Units.inchesToMeters(265.74),
                                            Units.inchesToMeters(27.38),
                                            new Rotation3d())),
                            new AprilTag(
                                    6,
                                    new Pose3d(
                                            Units.inchesToMeters(40.45),
                                            Units.inchesToMeters(174.19),
                                            Units.inchesToMeters(18.22),
                                            new Rotation3d())),
                            new AprilTag(
                                    7,
                                    new Pose3d(
                                            Units.inchesToMeters(40.45),
                                            Units.inchesToMeters(108.19),
                                            Units.inchesToMeters(18.22),
                                            new Rotation3d())),
                            new AprilTag(
                                    8,
                                    new Pose3d(
                                            Units.inchesToMeters(40.45),
                                            Units.inchesToMeters(42.19),
                                            Units.inchesToMeters(18.22),
                                            new Rotation3d()))),
                    fieldLength,
                    fieldWidth);
}