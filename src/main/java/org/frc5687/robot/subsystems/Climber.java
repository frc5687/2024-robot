package org.frc5687.robot.subsystems;

import org.frc5687.lib.drivers.OutliersTalon;
// import org.frc5687.lib.sensors.HallEffect;
import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotMap;
import org.frc5687.robot.util.OutliersContainer;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.motorcontrol.Talon;

public class Climber extends OutliersSubsystem {

    private OutliersTalon _talon;
    // private HallEffect _climbEffect;
    
    public Climber(OutliersContainer container) {
        super(container);
            _talon = new OutliersTalon(RobotMap.CAN.TALONFX.CLIMBER, Constants.Climber.CAN_BUS, "Climber");
            _talon.configure(Constants.Climber.CONFIG);
            _talon.configureClosedLoop(Constants.Climber.CLOSED_LOOP_CONFIG);
            _talon.setPosition(Constants.Climber.LOWER_LIMIT);
        // _climbEffect = new HallEffect(RobotMap.DIO.LOWER_CLIMB);
    }
 
    public void setPositionMeters(double meters) {
        if(meters < Constants.Climber.LOWER_LIMIT){
            warn("Attempted to set climber past lower limit.");
        } else if (meters > Constants.Climber.UPPER_LIMIT){
            warn("Attempted to set climber past upper limit.");
        } else{
        _talon.setMotionMagic(meters/(2 * Math.PI * Constants.Climber.WINCH_RADIUS));
        }
    }





    @Override
    public void updateDashboard() {} 
}
