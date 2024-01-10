/* Team 5687 (C)2022 */
package org.frc5687.lib.math;

import org.frc5687.robot.Constants;
import org.frc5687.robot.util.Helpers;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Vector2d {
    protected static final Vector2d IDENTITY = new Vector2d();

    public static Vector2d identity() {
        return IDENTITY;
    }

    protected double _x;
    protected double _y;

    public Vector2d() {
        _x = 0;
        _y = 0;
    }

    public Vector2d(double x, double y) {
        _x = x;
        _y = y;
    }

    public Vector2d(Vector2d other) {
        _x = other._x;
        _y = other._y;
    }

    public Vector2d(final Vector2d start, final Vector2d end) {
        _x = end._x - start._x;
        _y = end._y - start._y;
    }

    public void setX(double x) {
        _x = x;
    }

    public double x() {
        return _x;
    }

    public void setY(double y) {
        _y = y;
    }

    public double y() {
        return _y;
    }

    public double magnitude() {
        return Math.hypot(_x, _y);
    }

    public Vector2d normalize() {
        if (equals(new Vector2d())) return this;
        return scale(1.0 / magnitude());
    }

    public Rotation2d direction() {
        return new Rotation2d(_x, _y);
    }

    public Vector2d scale(double scalar) {
        return new Vector2d(_x * scalar, _y * scalar);
    }

    public Vector2d plus(Vector2d other) {
        return new Vector2d(_x + other._x, _y + other._y);
    }

    public Vector2d minus(Vector2d other) {
        return new Vector2d(_x - other._x, _y - other._y);
    }

    public double dot(Vector2d other) {
        return _x * other._x + _y * other._y;
    }

    public Vector2d inverse() {
        return new Vector2d(-_x, -_y);
    }

    public Vector2d interpolate(final Vector2d other, double x) {
        if (x <= 0) {
            return new Vector2d(this);
        } else if (x >= 1) {
            return new Vector2d(other);
        }
        return extrapolate(other, x);
    }

    public Vector2d extrapolate(final Vector2d other, double x) {
        return new Vector2d(x * (other._x - _x) + _x, x * (other._y - _y) + _y);
    }

    public boolean isWithinAngle(Vector2d A, Vector2d B, Vector2d C, boolean vertical) {
        Vector2d M = A.interpolate(C, 0.5); // midpoint
        Vector2d m = (new Vector2d(B, M)).normalize(); // mid-vector
        Vector2d a = (new Vector2d(B, A)).normalize(); // side vector
        Vector2d d = (new Vector2d(B, this)).normalize(); // vector to here
        if (vertical) {
            m = m.inverse();
            a = a.inverse();
        }
        return d.dot(m) > a.dot(m);
    }

    public boolean isWithinAngle(Vector2d A, Vector2d B, Vector2d C) {
        return isWithinAngle(A, B, C, false);
    }

    /** Assumes an angle centered at the origin. */
    public boolean isWithinAngle(Vector2d A, Vector2d C, boolean vertical) {
        return isWithinAngle(A, identity(), C, vertical);
    }

    public boolean isWithinAngle(Vector2d A, Vector2d C) {
        return isWithinAngle(A, C, false);
    }

    public Rotation2d angle(Vector2d other) {
        double cosAngle = dot(other) / (magnitude() * other.magnitude());
        if (Double.isNaN(cosAngle)) {
            return new Rotation2d();
        }
        return new Rotation2d(Math.acos(Math.min(1.0, Math.max(cosAngle, -1.0))));
    }

    public Translation2d toTranslation() {
        return new Translation2d(this._x, this._y);
    }

    public boolean equals(Vector2d other) {
        return Helpers.epsilonEquals(_x, other._x, Constants.EPSILON)
                && Helpers.epsilonEquals(_y, other._y, Constants.EPSILON);
    }
}

