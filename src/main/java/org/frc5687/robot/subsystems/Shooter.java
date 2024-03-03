package org.frc5687.robot.subsystems;

import org.frc5687.lib.cheesystuff.InterpolatingDouble;
import org.frc5687.lib.drivers.OutliersTalon;
import org.frc5687.robot.util.OutliersContainer;

import com.ctre.phoenix6.controls.Follower;

import edu.wpi.first.math.util.Units;

import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotMap;

public class Shooter extends OutliersSubsystem {
    private OutliersTalon _bottomTalon;
    private OutliersTalon _topTalon;
    private double _manualShootRPM = 3200;
    private boolean _spinUpAutomatically = true;

    public Shooter(OutliersContainer container) {
        super(container);
        _bottomTalon = new OutliersTalon(RobotMap.CAN.TALONFX.BOTTOM_SHOOTER, "CANivore", "Bottom Shooter");
        _topTalon = new OutliersTalon(RobotMap.CAN.TALONFX.TOP_SHOOTER, "CANivore", "Top Shooter");
        _bottomTalon.configure(Constants.Shooter.CONFIG);
        _topTalon.configure(Constants.Shooter.CONFIG);

        _bottomTalon.configureClosedLoop(Constants.Shooter.SHOOTER_CONTROLLER_CONFIG);
        _topTalon.setControl(new Follower(_bottomTalon.getDeviceID(), true));
        _bottomTalon.setConfigSlot(0);
        _topTalon.setConfigSlot(0);
    }

    public void setConfigSlot(int slot) {
        _bottomTalon.setConfigSlot(slot);
        _topTalon.setConfigSlot(slot);
    }

    public void setToIdle() {
        _bottomTalon.setVelocity(Constants.Shooter.IDLE_RPM);
    }

    public void setToPassthrough() {
        _bottomTalon.setVelocity(Constants.Shooter.PASSTHROUGH_RPM);
    }

    public void setToStop() {
        _bottomTalon.setVelocity(0);
    }

    public void setManualShootRPM(double speed) {
        _manualShootRPM = speed;
    }

    public void setToHandoffRPM(){
        _bottomTalon.setVelocity(Constants.Shooter.DUNKER_IN_RPM);
    }

    public void setToManualShootRPM() {
        _bottomTalon.setVelocity(_manualShootRPM);
    }

    public double getManualShootRPM() {
        return _manualShootRPM;
    }

    public void setPercentRPM(){
        _bottomTalon.setPercentOutput(.75);
    }

    public void setNegativePercentRPM(){
        _bottomTalon.setPercentOutput(-0.75);
    }

    /**
     * @param distance meters
     */
    public void setRPMFromDistance(double distance) {
        _bottomTalon.setVelocity(calculateRPMFromDistance(distance));
    }

    /**
     * Get the target RPM of the motor.
     * @return Motor angular velocity (RPM)
     */
    public double getTargetRPM() {
        return _bottomTalon.getClosedLoopReference().getValue() * 60; // rotations per second to RPM
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

    /**
     * TODO either remove this or move it to robotstate... either way rename it speaker mode amp mode seems nicer.
     * @return if we automatically spin up the shooter in idleshooter
     */
    public boolean getSpinUpAutomatically() {
        return _spinUpAutomatically;
    }

    /**
     * @param value true if we want to spin up automatically, false otherwise. this happens in idleshooter
     */
    public void setSpinUpAutomatically(boolean value) {
        _spinUpAutomatically = value;
    }

    public double calculateRPMFromDistance(double distance) {
        return Constants.Shooter.kRPMMap.getInterpolated(new InterpolatingDouble(distance)).value;
        // return Double.min(2800, Double.max(1700, Constants.Shooter.kRPMRegression.predict(distance)));
    }

    public void updateDashboard() {
        metric("Bottom Motor RPM", getBottomMotorRPM());
        metric("Top Motor RPM", getTopMotorRPM());
        metric("Manual Shoot RPM", getManualShootRPM());
        metric("Target RPM", getTargetRPM());
        metric("At Target RPM", isAtTargetRPM());
        metric("Spin Up Automatically?", getSpinUpAutomatically());
        metric("PID Output Shooter Voltage", _bottomTalon.getClosedLoopOutput().getValue());
    }
}
