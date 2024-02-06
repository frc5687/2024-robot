package org.frc5687.robot.subsystems;

import org.frc5687.lib.drivers.OutliersTalon;
// import org.frc5687.lib.sensors.HallEffect;
import org.frc5687.robot.Constants;
import org.frc5687.robot.OI;
import org.frc5687.robot.RobotMap;
import org.frc5687.robot.util.OutliersContainer;

// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.DoubleSolenoid;
// import edu.wpi.first.wpilibj.PneumaticsModuleType;
// import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
// import edu.wpi.first.wpilibj.motorcontrol.Talon;

public class Climber extends OutliersSubsystem{

    private OutliersTalon _talon;
    private OI _oi;
    // private DoubleSolenoid _ratchet;
    // private boolean _isRatchetUp;
    // private ClimberStep _step = ClimberStep.UNKNOWN;
    
    public Climber(OutliersContainer container) {
        super(container);
            _talon = new OutliersTalon(RobotMap.CAN.TALONFX.CLIMBER, Constants.Climber.CAN_BUS, "Climber");
            _talon.configure(Constants.Climber.CONFIG);
            _talon.configureClosedLoop(Constants.Climber.CLOSED_LOOP_CONFIG);
            _talon.setPosition(Constants.Climber.LOWER_LIMIT);
        
    //         _ratchet = new DoubleSolenoid(
    //             PneumaticsModuleType.REVPH,
    //             RobotMap.PCM.RATCHET_LOWER,
    //             RobotMap.PCM.RATCHET_RAISE
    //         );

    //         _isRatchetUp = true;
    }
 
    public void setPositionMeters(double meters) {
        if(meters < Constants.Climber.LOWER_LIMIT){
            warn("Attempted to set climber past lower limit.");
        } else if (meters > Constants.Climber.UPPER_LIMIT){
            warn("Attempted to set climber past upper limit.");
        } else{
        _talon.setMotionMagic(meters * Constants.Climber.CLIMBER_GEAR_RATIO /(2 * Math.PI * Constants.Climber.WINCH_RADIUS));
        }
    }

    public void setSpeed(double speed) {
        _talon.setPercentOutput(speed);
    }

    public double getMeters() {
        return (_talon.getPosition().getValue() / Constants.Climber.CLIMBER_GEAR_RATIO) * (2 * Math.PI * Constants.Climber.WINCH_RADIUS);
    }

    // public void changeRatchetUp(){
    //     //sets the ratchet to allow the climber to go up
    //     _ratchet.set(Value.kForward);
    //     info("Setting Ratchet To Up.");
    // }

    // public void changeRatchetDown(){
    //     //sets the ratchet to allow the climber to go down
    //     _ratchet.set(Value.kReverse);
    //     info("Setting Ratchet To Down.");
    // }

    // public boolean getClimberPosition() {
    //     return   
    // }

    // public void setStep(ClimberStep step){
    //     //Sets the current step of the climbing process
    //     _step = step;
    // }

    // public ClimberStep getStep() {
    //     return _step;
    // }

    // public enum ClimberStep { 
    //     UNKNOWN(0),
    //     STOW(1),
    //     STOWED(2),
    //     PREP_TO_CLIMB(3),
    //     READY_TO_CLIMB(4),
    //     RAISE_ARM(5),
    //     LOWER_ARM(6),
    //     ARM_LOWERED(7),
    //     DONE(8);

    //     private final int _value;
    //     ClimberStep(int value) { 
    //         _value = value; 
    //     }

    //     public int getValue() { 
    //         return _value; 
    //     }
    // }
    

    @Override
    public void updateDashboard() {
        metric("ClimberPosition", getMeters());        
    }
}
