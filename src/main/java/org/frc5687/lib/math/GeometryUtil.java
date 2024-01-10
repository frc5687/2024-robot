/* Team 5687 (C)2022 */
package org.frc5687.lib.math;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class GeometryUtil {

    public static Twist2d IDENTITY = new Twist2d(0.0, 0.0, 0.0);
    public static Rotation2d PI = new Rotation2d(Math.PI);

    public static Rotation2d getNearestPole(Rotation2d rot) {
        double pole_sin;
        double pole_cos;
        if (Math.abs(rot.getCos()) > Math.abs(rot.getSin())) {
            pole_cos = Math.signum(rot.getCos());
            pole_sin = 0.0;
        } else {
            pole_cos = 0.0;
            pole_sin = Math.signum(rot.getSin());
        }
        return new Rotation2d(pole_cos, pole_sin);
    }

    public static Rotation2d inverse(Rotation2d rot) {
        return new Rotation2d(rot.getCos(), -rot.getSin());
    }

    public static Rotation2d flip(Rotation2d rot) {
        return new Rotation2d(-rot.getCos(), -rot.getSin());
    }

    public static Twist2d toTwist2d(ChassisSpeeds chassisSpeeds) {
        return new Twist2d(
                chassisSpeeds.vxMetersPerSecond,
                chassisSpeeds.vyMetersPerSecond,
                chassisSpeeds.omegaRadiansPerSecond);
    }

    public static double getDistance(Rotation2d rot, Rotation2d other) {
        return inverse(rot).rotateBy(other).getRadians();
    }

    public static Vector2d rotationToVector(Rotation2d rot) {
        return new Vector2d(rot.getCos(), rot.getSin());
    }

    public static Vector2d translationToVector(Translation2d trans) {
        return new Vector2d(trans.getX(), trans.getY());
    }
}
