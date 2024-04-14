package org.frc5687.robot.subsystems;

import org.frc5687.lib.cheesystuff.InterpolatingDouble;
import org.frc5687.lib.drivers.OutliersTalon;
import org.frc5687.robot.util.OutliersContainer;

import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;

import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotMap;
import org.frc5687.robot.RobotState;

public class Shooter extends OutliersSubsystem {
    private OutliersTalon _bottomTalon;
    private OutliersTalon _topTalon;
    private double _manualShootRPM = 2350;
    private double _targetRPM = 0;
    private boolean _isAmpMode;
    private VelocityTorqueCurrentFOC _focVelocity;

    private RobotState _robotState = RobotState.getInstance();

    public Shooter(OutliersContainer container) {
        super(container);
        _bottomTalon = new OutliersTalon(RobotMap.CAN.TALONFX.BOTTOM_SHOOTER, "CANivore", "Bottom Shooter");
        _topTalon = new OutliersTalon(RobotMap.CAN.TALONFX.TOP_SHOOTER, "CANivore", "Top Shooter");

        _bottomTalon.configure(Constants.Shooter.BOTTOM_CONFIG);
        _topTalon.configure(Constants.Shooter.TOP_CONFIG);

        _bottomTalon.configureClosedLoop(Constants.Shooter.SHOOTER_CONTROLLER_CONFIG);
        _topTalon.configureClosedLoop(Constants.Shooter.SHOOTER_CONTROLLER_CONFIG);

        _bottomTalon.setConfigSlot(0);
        _topTalon.setConfigSlot(0);

        // _focVelocity = new VelocityTorqueCurrentFOC(0);
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

    public void setToPassthroughHarder() {
        setShooterMotorRPM(Constants.Shooter.PASSTHROUGH_RPM_HARDER);
    }

    public void setToStop() {
        setShooterMotorRPM(0);
    }

    public void setManualShootRPM(double rpm) {
        _manualShootRPM = rpm;
    }

    public void setToHandoffRPM(){
        setShooterMotorRPM(Constants.Shooter.DUNKER_IN_RPM);
    }

    public void setToManualShoot() {
        setShooterMotorRPM(_manualShootRPM);
    }

    public double getManualShootRPM() {
        return _manualShootRPM;
    }

    public void setToEject(){
        // _bottomTalon.setPercentOutput(Constants.Shooter.EJECT_PERCENT_OUTPUT);
    }

    public void setToIntakeEject(){
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
        return _targetRPM > 0 && Math.abs(_targetRPM - getCombinedRPM()) < _robotState.getToleranceFromVision();
    }

    public double getTargetRPM() {
        return _targetRPM;
    }

    
    public boolean isAtRPMMatch(double distance, double vxMetersPerSecond, Double vyMetersPerSecond) {
        double vMetersPerSecond = Math.pow((Math.pow(vxMetersPerSecond,2)+Math.pow(vyMetersPerSecond,2)),.5);
        //double predictedDistance = I did my math wrong hol up
        return _targetRPM > 0 && Math.abs(_targetRPM - getCombinedRPM()) < Constants.Shooter.MATCH_RPM_TOLERANCE*calculateRPMFromDistance(distance)+ _robotState.getToleranceFromVision() && Math.abs(_targetRPM - getCombinedRPM()) > Constants.Shooter.MATCH_RPM_TOLERANCE*calculateRPMFromDistance(distance) - _robotState.getToleranceFromVision();
    }

    public void setShooterMotorRPM(double rpm) {
        _targetRPM = rpm;
        // setVelocity does rpm to rps conversion
        _topTalon.setVelocity(rpm);
        _bottomTalon.setVelocity(rpm);
        // _bottomTalon.setControl(_focVelocity.withVelocity(rpm / 60.0));
    }

    /**
     * TODO either remove this or move it to robotstate... either way rename it speaker mode amp mode seems nicer.
     * @return if we automatically spin up the shooter in idleshooter
     */
    public boolean getAmpMode() {
        return _isAmpMode;
    }

    /**
     * @param value true if we want to spin up automatically, false otherwise. this happens in idleshooter
     */
    public void setAmpMode(boolean value) {
        _isAmpMode = value;
        
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
        metric("Speaker Mode", !getAmpMode()); // violet likes speakler mode to be green speaerker spe.rka spaerklaler mode speaeker spaerker mode 
    }
}
