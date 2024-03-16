package org.frc5687.robot.commands.Intake;

import org.frc5687.robot.Constants;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.Intake;

public class IndexNote extends OutliersCommand {

    private final Intake _intake;
    private IndexState _state;

    public IndexNote(Intake intake) {
        _intake = intake;
        addRequirements(_intake);
    }

    @Override
    public void initialize() {
        super.initialize();
        _state = IndexState.START;
        // Make sure when this is called again it doesn't start if the note is already indexed
        if (_intake.isNoteIndexed()) {
            _state = IndexState.FINISH;
        }
    }

    @Override
    public void execute() {
        switch (_state) {
            case START:
                _intake.setSpeed(Constants.Intake.INTAKE_SPEED);
                if (_intake.isMiddleDetected()) {
                    _intake.setSpeed(Constants.Intake.INDEX_SPEED);
                    _state = IndexState.NOTE_DETECTED;
                }
                break;

            case NOTE_DETECTED:
                if (_intake.isTopDetected() && !_intake.isBottomDetected()) {
                    if (_intake.isTopDetected() && _intake.isMiddleDetected()) {
                        _intake.setSpeed(0);
                        _state = IndexState.FINISH;
                    }
                    _intake.setSpeed(Constants.Intake.REVERSE_INDEX_SPEED);
                } else if (!_intake.isTopDetected() && _intake.isBottomDetected()) {
                    _intake.setSpeed(Constants.Intake.INDEX_SPEED);
                } else if (_intake.isTopDetected() && _intake.isBottomDetected()) {
                    _intake.setSpeed(0);
                    _state = IndexState.FINISH;
                } else {
                    _intake.setSpeed(Constants.Intake.INDEX_SPEED);
                }
                break;
            case FINISH:
                _intake.setSpeed(0);
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return _state == IndexState.FINISH;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        if (interrupted) {
            error("Index interrupted");
        }
        _intake.setSpeed(0);
    }

    public enum IndexState {
        START(0),
        NOTE_DETECTED(1),
        FINISH(2);

        private final int _value;

        IndexState(int value) {
            _value = value;
        }

        public int getValue() {
            return _value;
        }
    }
}