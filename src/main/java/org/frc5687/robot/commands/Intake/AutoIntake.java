package org.frc5687.robot.commands.Intake;

import org.frc5687.robot.Constants;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.Intake;

public class AutoIntake extends OutliersCommand {
    
    private final Intake _intake;
    private IndexState _state;

    public AutoIntake(Intake intake) {
        _intake = intake;
        addRequirements(_intake);
    }
    @Override
    public void initialize() {
        super.initialize();
        _state = IndexState.NO_NOTE_YET;
    }

    @Override
    public void execute() {
        switch(_state) {
            case NO_NOTE_YET:
                metric("STATE", 0);
                _intake.setSpeed(Constants.Intake.INTAKE_SPEED);
                if (_intake.isBottomDetected()) {
                    error("GOING TO BOTTOM_SENSOR_DETECTED");
                    _state = IndexState.BOTTOM_SENSOR_DETECTED;
                }
            break;
            case BOTTOM_SENSOR_DETECTED:
                metric("STATE", 1);
                _intake.setSpeed(Constants.Intake.INDEX_SPEED);
                if (_intake.isTopDetected()) {
                    error("GOING TO TOP_SENSOR_DETECTED. this should end the command");
                    _state = IndexState.TOP_SENSOR_DETECTED;
                }
                break;
            case TOP_SENSOR_DETECTED:
                metric("STATE", 2);
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
        error("Is interrupted "+ String.valueOf(interrupted));
        super.end(interrupted);
        _intake.setSpeed(0);
    }

    private enum IndexState {
        NO_NOTE_YET(0),
        BOTTOM_SENSOR_DETECTED(1),
        TOP_SENSOR_DETECTED(2);

        private final int _value;

        IndexState(int value) {
            _value = value;
        }

        public int getValue() {
            return _value;
        }
    }
}
