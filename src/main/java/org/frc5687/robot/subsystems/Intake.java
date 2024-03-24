
package org.frc5687.robot.subsystems;

import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotMap;
import org.frc5687.robot.util.OutliersContainer;
import org.frc5687.lib.drivers.OutliersTalon;
import org.frc5687.lib.sensors.ProximitySensor;

public class Intake extends OutliersSubsystem {

    public enum IndexState {
        IDLE(0),
        INTAKING(1),
        INDEXING(2),
        INDEXED(3);

        private final int _value;

        IndexState(int value) {
            _value = value;
        }

        public int getValue() {
            return _value;
        }
    }

    private final OutliersTalon _talon;
    private final ProximitySensor _top;
    private final ProximitySensor _bottom;
    private final ProximitySensor _mid;
    private boolean _autoIntakeFlag = false;
    private IndexState _indexState;

    public Intake(OutliersContainer container) {
        super(container);
        _talon = new OutliersTalon(RobotMap.CAN.TALONFX.INTAKE, Constants.Intake.CAN_BUS, "Intake");
        _talon.configure(Constants.Intake.CONFIG);
        _talon.configureClosedLoop(Constants.Intake.CLOSED_LOOP_CONFIG);
        _top = new ProximitySensor(RobotMap.DIO.TOP_DONUT_SENSOR);
        _bottom = new ProximitySensor(RobotMap.DIO.BOTTOM_DONUT_SENSOR);
        _mid = new ProximitySensor(RobotMap.DIO.MID_DONUT_SENSOR);
        _indexState = IndexState.IDLE;
    }
    
    public boolean isTopDetected(){
        return _top.get();
    }

    public boolean isBottomDetected() {
        return _bottom.get();
    }

    public boolean isMiddleDetected() {
        return _mid.get();
    }

    public void setSpeed(double intakeSpeed) {
        _talon.setPercentOutput(intakeSpeed);
    }

    public boolean isNoteDetected() {
        return isBottomDetected() || isTopDetected() || isMiddleDetected();
    }

    /* Checks if the two configurations for a note that is indexed is present */
    public boolean isNoteIndexed() {
        return (isMiddleDetected() && isTopDetected()) || (isBottomDetected() && isTopDetected()) || isMiddleDetected();
    }

    public void setAutoIntakeFlag(boolean flag) {
        _autoIntakeFlag = flag;
    }

    public boolean getAutoIntakeFlag() {
        return _autoIntakeFlag;
    }

    public IndexState getIndexState() {
        return _indexState;
    }

    public void setIndexState(IndexState intaking) {
        _indexState = intaking;
    }

    @Override
    public void updateDashboard() {
        metric("Top Detected", isTopDetected() ? 1000.0: 0.0);
        metric("Bottom Detected", isBottomDetected() ? 1000.0: 0.0);
        metric("Is note indexed", isNoteIndexed());
        metric("AutoIntake Flag", getAutoIntakeFlag());
    }
}