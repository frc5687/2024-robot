package org.frc5687.robot.subsystems;

import org.frc5687.lib.drivers.OutliersTalon;
import org.frc5687.lib.sensors.HallEffect;
import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotMap;
import org.frc5687.robot.util.OutliersContainer;

import edu.wpi.first.math.util.Units;

public class Deflector extends OutliersSubsystem {

    private OutliersTalon _talon;
    private HallEffect _lowerHall;
    private double _targetAngle;

    public Deflector(OutliersContainer container) {
        super(container);
        _talon = new OutliersTalon(RobotMap.CAN.TALONFX.DEFLECTOR, Constants.Deflector.CAN_BUS, "Deflector");
        _talon.configure(Constants.Deflector.CONFIG);
        _talon.configureClosedLoop(Constants.Deflector.CLOSED_LOOP_CONFIG);

        _lowerHall = new HallEffect(RobotMap.DIO.LOWER_HALL);
    }

    @Override
    public void periodic() {
        if (_lowerHall.get()) {
            setEncoderAngle(Constants.Deflector.LOWER_HALL_ANGLE);
        }
    }

    public double getAngle() {
        return Units.rotationsToRadians(_talon.getPosition().getValue()) / Constants.Deflector.GEAR_RATIO;
    }

    public double getTargetAngle() {
        return _targetAngle;
    }

    public void setEncoderAngle(double angle) {
        _talon.setPosition(Units.radiansToRotations(angle) * Constants.Deflector.GEAR_RATIO);
    }

    public void setTargetAngle(double angle) {
        if (angle < Constants.Deflector.LOWER_HALL_ANGLE) {
            warn("Attempted to set deflector past lower limit.");
            setTargetAngle(Constants.Deflector.LOWER_HALL_ANGLE);
        } else if (angle > Constants.Deflector.UPPER_HALL_ANGLE) {
            warn("Attempted to set deflector past upper limit.");
            setTargetAngle(Constants.Deflector.UPPER_HALL_ANGLE);
        } else {
            _targetAngle = angle;
            info("Setting Deflector angle to " + _targetAngle);
            _talon.setMotionMagic(Units.radiansToRotations(angle) * Constants.Deflector.GEAR_RATIO);
        }
    }

    //don't use this for non-zero purposes!!
    public void beginDeflectorZero() {
        _talon.setPercentOutput(-0.1);
    }

    public void stopMotor() {
        _talon.setPercentOutput(0.0);
    }
    
    public boolean getHall() {
        return _lowerHall.get();
    }

    public boolean isAtTargetAngle() {
        return Math.abs(_targetAngle - getAngle()) < Constants.Deflector.ANGLE_TOLERANCE;
    }

    public double calculateAngleFromDistance(double distance) {
        return Constants.Shooter.kDeflectorRegression.predict(distance);
    }

    @Override
    public void updateDashboard() {
        metric("Deflector Target Angle", getTargetAngle());
        metric("Deflector Current Angle", getAngle());
        metric("Deflector Hall", getHall());
    }
}