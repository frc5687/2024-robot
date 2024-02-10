package org.frc5687.robot.commands.Climber;

import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.Climber;
import org.frc5687.robot.Constants;
public class DriveForward extends OutliersCommand {
    
    private Climber _climber;
    private Step _step = Step.START;

    public DriveForward(Climber climber) {
        _climber = climber;
        addRequirements(_climber);
        info("Instantiated DriveForward Check");

    }

    @Override
    public void initialize(){
        super.initialize();
        info("Initialized DriveForward");
        _climber.setStep(Climber.ClimberStep.DRIVE_FORWARD);
        _step = Step.START;
    }

    @Override
    public void execute() {
        super.execute();
        switch(_step) {
            case START:
                break;
            case DRIVE_FORWARD:
                break;
            case DONE:
                break;
        }
    }

    @Override
    public boolean isFinished() {
        super.isFinished();
        return true;
    }

    public enum Step {
        START(0),
        DRIVE_FORWARD(1),
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
