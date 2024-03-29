package org.frc5687.robot.subsystems;

import org.frc5687.lib.drivers.OutliersTalon;
import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotMap;
import org.frc5687.robot.util.OutliersContainer;

public class Climber extends OutliersSubsystem {

    private OutliersTalon _talon;
    private ClimberStep _step = ClimberStep.STOWED;
    
    public Climber(OutliersContainer container) {
        super(container);
        _talon = new OutliersTalon(RobotMap.CAN.TALONFX.CLIMBER, Constants.Climber.CAN_BUS, "Climber");
        _talon.configure(Constants.Climber.CONFIG);
        _talon.configureClosedLoop(Constants.Climber.CLOSED_LOOP_CONFIG);
        _talon.setPosition(Constants.Climber.ZERO_VALUE);
    }
 
    public void setPositionMeters(double meters) {
        //flipped < & > because the climber is inverted.
        if (meters > Constants.Climber.LOWER_LIMIT) {
            warn("Attempted to set climber past lower limit.");
        } else if (meters < Constants.Climber.UPPER_LIMIT){
            warn("Attempted to set climber past upper limit.");
        } else {
            metric("target meters", meters);
            _talon.setMotionMagic((meters * Constants.Climber.CLIMBER_GEAR_RATIO) / (2 * Math.PI * Constants.Climber.WINCH_RADIUS));
        }
    }

    public void setSpeed(double speed) {
        _talon.setPercentOutput(speed);
    }

    public double getPositionMeters() {
        return (_talon.getPosition().getValue() / Constants.Climber.CLIMBER_GEAR_RATIO) * (2 * Math.PI * Constants.Climber.WINCH_RADIUS);
    }

    public void setStep(ClimberStep step) {
        //Sets the current step of the climbing process
        _step = step;
    }

    public ClimberStep getStep() {
        return _step;
    }

    public enum ClimberStep { 
        STOWED(0),
        CLEARING_DUNKER(1),
        RAISING(2),
        RAISED(3),
        STOWING(4);

        private final int _value;
        ClimberStep(int value) { 
            _value = value; 
        }

        public int getValue() { 
            return _value; 
        }
    }
    

    @Override
    public void updateDashboard() {
        metric("Climber Position", getPositionMeters());
        metric("Climber Step", getStep().toString());     
    }
}
