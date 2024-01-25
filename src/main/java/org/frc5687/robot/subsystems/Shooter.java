package org.frc5687.robot.subsystems;

import org.frc5687.lib.drivers.OutliersTalon;
import org.frc5687.robot.util.OutliersContainer;

import com.ctre.phoenix6.controls.Follower;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotMap;

public class Shooter extends OutliersSubsystem {
    public OutliersTalon _bottomTalon;
    public OutliersTalon _topTalon;
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

    public void setTargetRPM(double speed) {
        _targetRPM = speed;
    }

    public double getTargetRPM() {
        return _targetRPM;
    }

    public void enableMotor() {
        _bottomTalon.setVelocity(/*_targetRPM*/1200);
    }

    public double getMotorRPM() {
        return _topTalon.getVelocity().getValueAsDouble();
    }

    public void disableMotor() {
        // TODO: coast
        _bottomTalon.setVelocity(0);
    }

    public void updateDashboard() {
        SmartDashboard.putNumber("_bottomTalon RPM Shooter", _bottomTalon.getVelocity().getValue() * 60);
        SmartDashboard.putNumber("Targt RPM Shooter", _targetRPM);
    }
}
