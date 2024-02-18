package org.frc5687.robot.commands.Intake;

import org.frc5687.robot.Constants;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.Intake;

public class AutoIntake extends OutliersCommand {
    
    private final Intake _intake;
    private IndexState _state;

    public AutoIntake(Intake intake) {
        _intake = intake;
        _state = IndexState.NO_NOTE_YET;
        addRequirements(_intake);
    }
    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        switch(_state) {
            case NO_NOTE_YET:
                _intake.setSpeed(Constants.Intake.INTAKE_SPEED);
                if (_intake.isBottomDetected()) {
                    _intake.setSpeed(0.5); // FIXME: random untested value
                    error("Going to Bottom Sensor Detected");
                    _state = IndexState.BOTTOM_SENSOR_DETECTED;
                }
            break;
            case BOTTOM_SENSOR_DETECTED:
                if (_intake.isTopDetected()) {
                    _intake.setSpeed(0.0);
                    error("Going to Top Sensor Detected");
                    _state = IndexState.TOP_SENSOR_DETECTED;
                } else {
                    _intake.setSpeed(Constants.Intake.INDEX_SPEED);
                }
                break;
            case TOP_SENSOR_DETECTED:
                if (_intake.isBottomDetected()) {
                    _intake.setSpeed(0.0);
                    error("Going to Bottom Sensor Again");
                    _state = IndexState.BOTTOM_SENSOR_AGAIN;
                } else {
                    _intake.setSpeed(Constants.Intake.INDEX_SPEED);
                }
                break;
            case BOTTOM_SENSOR_AGAIN:
                error("Stopping");
                _intake.setSpeed(0.0);
                break;
            default:
                break;
            
        }
    }

    @Override
    public boolean isFinished() {
       return _state == IndexState.BOTTOM_SENSOR_AGAIN;
    }

    @Override
    public void end(boolean interrupted) {
        error("Is interrupted "+ String.valueOf(interrupted));
        super.end(interrupted);
        _intake.setSpeed(0);
    }

    private enum IndexState {
        NO_NOTE_YET(0),
        BOTTOM_SENSOR_DETECTED(1),
        TOP_SENSOR_DETECTED(2),
        BOTTOM_SENSOR_AGAIN(3);

        private final int _value;

        IndexState(int value) {
            _value = value;
        }

        public int getValue() {
            return _value;
        }
    }
}
