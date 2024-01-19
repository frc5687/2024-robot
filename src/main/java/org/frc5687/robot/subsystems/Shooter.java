package org.frc5687.robot.subsystems;

import org.frc5687.lib.drivers.OutliersTalon;
import org.frc5687.robot.util.OutliersContainer;

import com.ctre.phoenix6.controls.Follower;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotMap;

public class Shooter extends OutliersSubsystem {
    public OutliersTalon _leftTalon;
    public OutliersTalon _rightTalon;
    private double _targetRPM = 0;
    public Shooter(OutliersContainer container) {
        super(container);
        _leftTalon = new OutliersTalon(RobotMap.CAN.TALONFX.LEFT_SHOOTER, "CANivore", "Left Shooter");
        _rightTalon = new OutliersTalon(RobotMap.CAN.TALONFX.RIGHT_SHOOTER, "CANivore", "Right Shooter");
        _leftTalon.configure(Constants.Shooter.CONFIG);
        _rightTalon.configure(Constants.Shooter.CONFIG);

        _leftTalon.configureClosedLoop(Constants.Shooter.SHOOTER_CONTROLLER_CONFIG);
        _rightTalon.setControl(new Follower(_leftTalon.getDeviceID(), true));
    }

    public void setTargetRPM(double speed) {
        _targetRPM = speed;
    }

    public double getTargetRPM() {
        return _targetRPM;
    }

    public void enableMotor() {
        _leftTalon.setVelocity(_targetRPM);
    }

    public void disableMotor() {
        // TODO: coast
        _leftTalon.setVelocity(0);
    }

    public void updateDashboard() {
        SmartDashboard.putNumber("_leftTalon RPM Shooter", _leftTalon.getVelocity().getValue() * 60);
        SmartDashboard.putNumber("Targt RPM Shooter", _targetRPM);
    }
}
