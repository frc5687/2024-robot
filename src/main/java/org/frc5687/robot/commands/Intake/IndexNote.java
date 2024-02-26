package org.frc5687.robot.commands.Intake;

import org.frc5687.robot.Constants;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.Intake;

public class IndexNote extends OutliersCommand {
    
    private final Intake _intake;
    private IndexState _state;

    public IndexNote(Intake intake) {
        _intake = intake;
        _state = IndexState.BOTTOM_SENSOR_DETECTED;
        addRequirements(_intake);
    }
    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        switch(_state) {
            case BOTTOM_SENSOR_DETECTED:
                if (_intake.isTopDetected()) {
                    _intake.setSpeed(0.0);
                    _state = IndexState.TOP_SENSOR_DETECTED;
                } else {
                    _intake.setSpeed(Constants.Intake.INDEX_SPEED);
                }
                break;
            case TOP_SENSOR_DETECTED:
                _intake.setSpeed(0.0); // doubt this will run, but the end() should work.
                break;
            default:
                break;
            
        }
    }

    @Override
    public boolean isFinished() {
       return _state == IndexState.TOP_SENSOR_DETECTED;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        _intake.setSpeed(0);
    }

    public enum IndexState {
        BOTTOM_SENSOR_DETECTED(0),
        TOP_SENSOR_DETECTED(1);

        private final int _value;

        IndexState(int value) {
            _value = value;
        }

        public int getValue() {
            return _value;
        }
    }
}
