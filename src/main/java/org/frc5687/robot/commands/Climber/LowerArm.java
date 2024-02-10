package org.frc5687.robot.commands.Climber;

import org.frc5687.robot.Constants;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.Climber;

public class LowerArm extends OutliersCommand {
    private Climber _climber;
    private Step _step = Step.START;

    public LowerArm(Climber climber) {
        _climber = climber;
        addRequirements(_climber);
        info("Instantiated LowerArm Check");
    }

    @Override
    public void initialize() {
        super.initialize();
        info("Initialized LowerArm");
        _climber.setStep(Climber.ClimberStep.LOWER_ARM);
        _step = Step.START;
    }

    @Override
    public void execute() {
        super.execute();
        switch(_step) {
            case START:
                _step = Step.LOWER_ARM;
                info("LowerArm advancing to LOWER_ARM step.");
                break;
            case LOWER_ARM:
                _climber.setPositionMeters(Constants.Climber.LOWER_LIMIT);
                info("LowerArm advancing to DONE step");
            case DONE:
                break;
        }
    }

    @Override
    public boolean isFinished() {
        super.isFinished();
        if ( _step == Step.DONE && _climber.isClimberAtLowerLimit()) {
            _climber.setStep(Climber.ClimberStep.ARM_LOWERED);
            info("Finished LowerArm.");
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
        LOWER_ARM(1),
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
