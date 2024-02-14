package org.frc5687.robot.subsystems;

import org.frc5687.lib.drivers.OutliersTalon;
import org.frc5687.robot.util.OutliersContainer;

import com.ctre.phoenix6.controls.Follower;


import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotMap;

public class Shooter extends OutliersSubsystem {
    private OutliersTalon _bottomTalon;
    private OutliersTalon _topTalon;
    private double _targetRPM = 0;
    public Shooter(OutliersContainer container) {
        super(container);
        _bottomTalon = new OutliersTalon(RobotMap.CAN.TALONFX.BOTTOM_SHOOTER, "CANivore", "Left Shooter");
        _topTalon = new OutliersTalon(RobotMap.CAN.TALONFX.TOP_SHOOTER, "CANivore", "Right Shooter");
        _bottomTalon.configure(Constants.Shooter.CONFIG);
        _topTalon.configure(Constants.Shooter.CONFIG);

        _bottomTalon.configureClosedLoop(Constants.Shooter.SHOOTER_CONTROLLER_CONFIG);
        _topTalon.setControl(new Follower(_bottomTalon.getDeviceID(), true));
    }

    public void setIdle() {
        _bottomTalon.setVelocity(Constants.Shooter.IDLE_RPM);
    }

    public void setTargetRPM(double speed) {
        _targetRPM = speed;
    }

    public void setToTarget(){
        _bottomTalon.setVelocity(_targetRPM);
    }

    public double getTargetRPM() {
        return _targetRPM;
    }

    public double getMotorRPM() {
        return OutliersTalon.rotationsPerSecToRPM(_bottomTalon.getVelocity().getValueAsDouble(), 1);
    }

    public boolean isAtTargetRPM(){
        return getTargetRPM() > 0 && Math.abs(getTargetRPM() - getMotorRPM()) < Constants.Shooter.VELOCITY_TOLERANCE;
    }

    public double calculateRPMFromDistance(double distance) {
        return Constants.Shooter.kRPMRegression.predict(distance);
    }

    public void updateDashboard() {
        metric("Shooter RPM", getMotorRPM());
        metric("Target RPM", _targetRPM);
        metric("At Target RPM", isAtTargetRPM());
    }
}
