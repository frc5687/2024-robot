package org.frc5687.robot.commands.Climber;

import org.frc5687.robot.Constants;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.Climber;
import org.frc5687.robot.subsystems.Climber.ClimberStep;

import edu.wpi.first.wpilibj.Relay.Value;

public class RaiseArm extends OutliersCommand {
    
    private Climber _climber;
    private Step _step = Step.START;

    public RaiseArm (Climber climber) {
        _climber = climber;
        addRequirements(_climber);
        info("Instantiated RaiseArm Check");
    }

    @Override
    public void initialize(){
        super.initialize();
        info("Initialized RaiseArm");
        _climber.setStep(Climber.ClimberStep.RAISE_ARM);
        _step = Step.START;
    }

    @Override
    public void execute() {
        super.execute();
        switch(_step) {
            case START:
                _step = Step.RAISE_ARM;
                info("RaiseArm advancing to RAISE_ARM step.");
                break;
            case RAISE_ARM:
                _climber.setPositionMeters(Constants.Climber.UPPER_LIMIT);
                _step = Step.DONE;
                info("RaiseArm advancing to DONE step.");
                break;
            case DONE:
                break;
        }
    }

    @Override
    public boolean isFinished() {
        super.isFinished();

        if ( _step == Step.DONE && _climber.isClimberAtUpperLimit()) {
            _climber.setStep(Climber.ClimberStep.ARM_RAISED);
            info("Finished RaiseArm.");
            return true;
        }
        else{
            return false;
        }
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
    
    public enum Step {
        START(0),
        RAISE_ARM(1),
        DONE(2);

        private final int _value;
        Step(int value) { 
            _value = value; 
        }

        public int getValue() { 
            return _value; 
        }
    }
}
