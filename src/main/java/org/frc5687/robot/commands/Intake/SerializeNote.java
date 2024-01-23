package org.frc5687.robot.commands.Intake;

import org.frc5687.robot.Constants;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.Intake;

public class SerializeNote extends OutliersCommand {
    
    private final Intake _intake;
    private SerializeState _state;
    public SerializeNote(Intake intake) {
        _intake = intake;
        _state = SerializeState.DETECTED_NOTE;
    }
    @Override
    public void initialize() {
        super.initialize();
        _state = SerializeState.DETECTED_NOTE;
    }

    @Override
    public void execute() {
        switch(_state) {
            case DETECTED_NOTE:
                if (!_intake.isDonutDetected() && !_intake.isBottomDetected()) {
                    _intake.setSpeed(0.0);
                    _state = SerializeState.BOTH_SENSORS_CLEARED;
                } else {
                    _intake.setSpeed(Constants.Intake.FORWARD_SERIALIZE_SPEED);
                }
                break;
            case BOTH_SENSORS_CLEARED:
                if (_intake.isDonutDetected() && !_intake.isBottomDetected()) {
                    _intake.setSpeed(0.0);
                    _state = SerializeState.NOTE_SERIALIZED;
                } else {
                    _intake.setSpeed(Constants.Intake.REVERSE_SERIALIZE_SPEED);
                }
                break;
            case NOTE_SERIALIZED:
                    _intake.setSpeed(0.0);
                break;  
            default:
                break;
        }
        _intake.setSpeed(Constants.Intake.INTAKE_SPEED);
    }

    @Override
    public boolean isFinished() {
        return _state == SerializeState.NOTE_SERIALIZED;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        _intake.setSpeed(0);
    }

    public enum SerializeState {
        DETECTED_NOTE(0),
        BOTH_SENSORS_CLEARED(1),
        NOTE_SERIALIZED(2);

        private final int _value;

        SerializeState(int value) {
            _value = value;
        }

        public int getValue() {
            return _value;
        }
    }
}
