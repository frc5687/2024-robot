package org.frc5687.robot.subsystems;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.frc5687.lib.cheesystuff.InterpolatingDouble;
import org.frc5687.lib.drivers.OutliersTalon;
import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotMap;
import org.frc5687.robot.util.OutliersContainer;

import com.ctre.phoenix6.controls.Follower;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Shooter extends OutliersSubsystem {
    private OutliersTalon _bottomTalon;
    private OutliersTalon _topTalon;
    private double _targetRPM = 0;

    // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
    private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
    // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
    private final MutableMeasure<Angle> m_angle = mutable(Rotations.of(0));
    // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
    private final MutableMeasure<Velocity<Angle>> m_velocity = mutable(RotationsPerSecond.of(0));

    private SysIdRoutine _routine;

    public Shooter(OutliersContainer container) {
        super(container);
        _bottomTalon = new OutliersTalon(RobotMap.CAN.TALONFX.BOTTOM_SHOOTER, "CANivore", "Bottom Shooter");
        _topTalon = new OutliersTalon(RobotMap.CAN.TALONFX.TOP_SHOOTER, "CANivore", "Top Shooter");
        _bottomTalon.configure(Constants.Shooter.CONFIG);
        _topTalon.configure(Constants.Shooter.CONFIG);

        _bottomTalon.configureClosedLoop(Constants.Shooter.SHOOTER_CONTROLLER_CONFIG);
        _topTalon.setControl(new Follower(_bottomTalon.getDeviceID(), true));

        _routine = new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                (Measure<Voltage> volts) -> {
                    _bottomTalon.setVoltage(volts.in(Volts));
                },
                log -> {
                    // Record a frame for the shooter motor.
                    log.motor("shooter-wheel")
                        .voltage(
                            m_appliedVoltage.mut_replace(_bottomTalon.getMotorVoltage().getValueAsDouble(), Volts))
                        .angularPosition(m_angle.mut_replace(_bottomTalon.getRotorPosition().getValueAsDouble(), Rotations))
                        .angularVelocity(m_velocity.mut_replace(_bottomTalon.getRotorVelocity().getValueAsDouble(), RotationsPerSecond));
                },
                this
            )
        );
    }

    public void setToIdle() {
        _bottomTalon.setVelocity(Constants.Shooter.IDLE_RPM);
    }

    public void setToStop() {
        _bottomTalon.setVelocity(0);
    }

    public void setToDunkInRPM() {
        _bottomTalon.setVelocity(Constants.Shooter.DUNKER_IN_RPM);
    }

    public void setToDunkOutRPM() {
        _bottomTalon.setVelocity(Constants.Shooter.DUNKER_OUT_RPM);
    }

    public void setTargetRPM(double speed) {
        _targetRPM = speed;
    }

    public void setToTarget() {
        _bottomTalon.setVelocity(_targetRPM);
    }

    public void setWithVoltage(double voltage) {
        _bottomTalon.setVoltage(voltage);
    }

    public double getTargetRPM() {
        return _targetRPM;
    }

    public double getBottomMotorRPM() {
        return OutliersTalon.rotationsPerSecToRPM(_bottomTalon.getVelocity().getValueAsDouble(), 1);
    }

    public double getTopMotorRPM() {
        return OutliersTalon.rotationsPerSecToRPM(_bottomTalon.getVelocity().getValueAsDouble(), 1);
    }

    public boolean isAtTargetRPM() {
        return getTargetRPM() > 0
                && Math.abs(getTargetRPM() - getBottomMotorRPM()) < Constants.Shooter.VELOCITY_TOLERANCE;
    }

    public double calculateRPMFromDistance(double distance) {
        return Constants.Shooter.kRPMMap.getInterpolated(new InterpolatingDouble(distance)).value;
        // return Double.min(2800, Double.max(1700, Constants.Shooter.kRPMRegression.predict(distance)));
    }

    public void updateDashboard() {
        metric("Bottom Motor RPM", getBottomMotorRPM());
        metric("Top Motor RPM", getTopMotorRPM());
        metric("Target RPM", getTargetRPM());
        metric("At Target RPM", isAtTargetRPM());
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return _routine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return _routine.dynamic(direction);
    }
}
