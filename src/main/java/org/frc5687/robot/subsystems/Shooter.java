package org.frc5687.robot.subsystems;

import org.frc5687.lib.cheesystuff.InterpolatingDouble;
import org.frc5687.lib.drivers.OutliersTalon;
import org.frc5687.robot.util.OutliersContainer;

import com.ctre.phoenix6.controls.Follower;

import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotMap;

public class Shooter extends OutliersSubsystem {
    private OutliersTalon _bottomTalon;
    private OutliersTalon _topTalon;
    private double _targetRPM = 0;
    private boolean _autoShootFlag = false;

    public Shooter(OutliersContainer container) {
        super(container);
        _bottomTalon = new OutliersTalon(RobotMap.CAN.TALONFX.BOTTOM_SHOOTER, "CANivore", "Bottom Shooter");
        _topTalon = new OutliersTalon(RobotMap.CAN.TALONFX.TOP_SHOOTER, "CANivore", "Top Shooter");
        _bottomTalon.configure(Constants.Shooter.CONFIG);
        _topTalon.configure(Constants.Shooter.CONFIG);

        _bottomTalon.configureClosedLoop(Constants.Shooter.SHOOTER_CONTROLLER_CONFIG);
        _topTalon.setControl(new Follower(_bottomTalon.getDeviceID(), true));
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

    public void setAutoShootFlag(boolean flag) {
        _autoShootFlag = flag;
    }

    public boolean getAutoShootFlag() {
        return _autoShootFlag;
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
        metric("AutoShoot Flag", getAutoShootFlag());
    }
}
