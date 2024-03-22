package org.frc5687.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import org.frc5687.lib.cheesystuff.InterpolatingDouble;
import org.frc5687.lib.drivers.OutliersTalon;
import org.frc5687.robot.util.OutliersContainer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotMap;

public class Shooter extends OutliersSubsystem {
    private OutliersTalon _bottomTalon;
    private OutliersTalon _topTalon;
    private double _manualShootRPM = 3200;
    private double _targetRPM = 0;
    private boolean _spinUpAutomatically = true;
    private VelocityTorqueCurrentFOC _focVelocity;
    private final VoltageOut _sysidControl = new VoltageOut(0);
    private SysIdRoutine _sysIdRoutine;

    public Shooter(OutliersContainer container) {
        super(container);
        SignalLogger.setPath("/home/lvuser/logs");
        SignalLogger.start();
        _bottomTalon = new OutliersTalon(RobotMap.CAN.TALONFX.BOTTOM_SHOOTER, "CANivore", "Bottom Shooter");
        _topTalon = new OutliersTalon(RobotMap.CAN.TALONFX.TOP_SHOOTER, "CANivore", "Top Shooter");

        _bottomTalon.configure(Constants.Shooter.BOTTOM_CONFIG);
        _topTalon.configure(Constants.Shooter.TOP_CONFIG);

        _bottomTalon.configureClosedLoop(Constants.Shooter.SHOOTER_CONTROLLER_CONFIG);
        _topTalon.configureClosedLoop(Constants.Shooter.SHOOTER_CONTROLLER_CONFIG);

        _bottomTalon.setConfigSlot(0);
        _topTalon.setConfigSlot(0);

        BaseStatusSignal.setUpdateFrequencyForAll(250,
                _bottomTalon.getPosition(),
                _bottomTalon.getVelocity(),
                _bottomTalon.getMotorVoltage());

        _bottomTalon.optimizeBusUtilization();

        _focVelocity = new VelocityTorqueCurrentFOC(0);

        _sysIdRoutine= new SysIdRoutine(
                new SysIdRoutine.Config(
                        null, // Default ramp rate is acceptable
                        Volts.of(4), // Reduce dynamic voltage to 4 to prevent motor brownout
                        null, // Default timeout is acceptable
                              // Log state with Phoenix SignalLogger class
                        (state) -> SignalLogger.writeString("state", state.toString())),
                new SysIdRoutine.Mechanism(
                        (Measure<Voltage> volts) -> setVolts(volts),
                        null,
                        this));

    }

    public void setConfigSlot(int slot) {
        _bottomTalon.setConfigSlot(slot);
        _topTalon.setConfigSlot(slot);
    }

    public void setToIdle() {
        setShooterMotorRPM(Constants.Shooter.IDLE_RPM);
    }

    public void setToPassRPM() {
        setShooterMotorRPM(Constants.Shooter.PASS_RPM);
    }

    public void setToPassthrough() {
        setShooterMotorRPM(Constants.Shooter.PASSTHROUGH_RPM);
    }

    public void setToStop() {
        setShooterMotorRPM(0);
    }

    public void setManualShootRPM(double rpm) {
        _manualShootRPM = rpm;
    }

    public void setToHandoffRPM() {
        setShooterMotorRPM(Constants.Shooter.DUNKER_IN_RPM);
    }

    public void setToManualShoot() {
        setShooterMotorRPM(_manualShootRPM);
    }

    public double getManualShootRPM() {
        return _manualShootRPM;
    }

    public void setToEject() {
        // _bottomTalon.setPercentOutput(Constants.Shooter.EJECT_PERCENT_OUTPUT);
    }

    public void setToIntakeEject() {
        // _bottomTalon.setPercentOutput(-Constants.Shooter.EJECT_PERCENT_OUTPUT);
    }

    /**
     * @param distance meters
     */
    public void setRPMFromDistance(double distance) {
        setShooterMotorRPM(calculateRPMFromDistance(distance));
    }

    public double getBottomMotorRPM() {
        return OutliersTalon.rotationsPerSecToRPM(_bottomTalon.getVelocity().getValueAsDouble(), 1);
    }

    public double getTopMotorRPM() {
        return OutliersTalon.rotationsPerSecToRPM(_bottomTalon.getVelocity().getValueAsDouble(), 1);
    }

    public double getCombinedRPM() {
        return (getBottomMotorRPM() + getTopMotorRPM()) / 2.0;
    }

    public boolean isAtTargetRPM() {
        return _targetRPM > 0 && Math.abs(_targetRPM - getCombinedRPM()) < Constants.Shooter.VELOCITY_TOLERANCE;
    }

    public void setShooterMotorRPM(double rpm) {
        _targetRPM = rpm;
        _topTalon.setControl(_focVelocity.withVelocity(rpm / 60.0));
        _bottomTalon.setControl(_focVelocity.withVelocity(rpm / 60.0));
    }

    /**
     * TODO either remove this or move it to robotstate... either way rename it
     * speaker mode amp mode seems nicer.
     * 
     * @return if we automatically spin up the shooter in idleshooter
     */
    public boolean getSpinUpAutomatically() {
        return _spinUpAutomatically;
    }

    /**
     * @param value true if we want to spin up automatically, false otherwise. this
     *              happens in idleshooter
     */
    public void setSpinUpAutomatically(boolean value) {
        _spinUpAutomatically = value;
    }

    public double calculateRPMFromDistance(double distance) {
        return Constants.Shooter.kRPMMap.getInterpolated(new InterpolatingDouble(distance)).value;
    }

    public void updateDashboard() {
        metric("Bottom Motor RPM", getBottomMotorRPM());
        metric("Top Motor RPM", getTopMotorRPM());
        metric("Manual Shoot RPM", getManualShootRPM());
        metric("Botton Talon Output", _bottomTalon.getClosedLoopOutput().getValueAsDouble());
        metric("Average Output", getCombinedRPM());
        metric("Target RPM", _targetRPM);
        metric("At Target RPM", isAtTargetRPM());
        metric("Spin Up Automatically?", getSpinUpAutomatically());
    }

    public void setVolts(Measure<Voltage> volts) {
        _bottomTalon.setControl(_sysidControl.withOutput(volts.in(Volts)));
        _topTalon.setControl(_sysidControl.withOutput(volts.in(Volts)));
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return _sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return _sysIdRoutine.dynamic(direction);
    }

}
