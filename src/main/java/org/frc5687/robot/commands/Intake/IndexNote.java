package org.frc5687.robot.commands.Intake;

import java.util.Optional;

import javax.swing.text.html.Option;

import org.frc5687.robot.Constants;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.Intake;

public class IndexNote extends OutliersCommand {
    
    private final Intake _intake;
    private IndexState _state;
    // private Optional<Long> _startedGoodTimestamp;
    // private double _targetPosition;

    public IndexNote(Intake intake) {
        _intake = intake;
        addRequirements(_intake);
    }
    @Override
    public void initialize() {
        super.initialize();
        _state = IndexState.START;
    }

    @Override
    public void execute() {
        switch (_state) {
            case START:
            _intake.setSpeed(1.0);
            if(_intake.isTopDetected()) {
                _state = IndexState.TOP_DETECTED;
            }
            break;
            case TOP_DETECTED:
            if (!_intake.isTopDetected()) {
                _intake.setSpeed(-0.2);
                _state = IndexState.TOP_NOT_DETECTED;
            }
            break;
            case TOP_NOT_DETECTED:
            if (_intake.isTopDetected()) {
                _intake.setSpeed(0);
                _state = IndexState.FINISH;
            }
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
        error("Intake interrupt" + interrupted);
        _intake.setSpeed(0);
    }

    public enum IndexState {
        START(0),
        TOP_DETECTED(1),
        TOP_NOT_DETECTED(2),
        FINISH(3);

        private final int _value;

        IndexState(int value) {
            _value = value;
        }

        public int getValue() {
            return _value;
        }
    }
}
