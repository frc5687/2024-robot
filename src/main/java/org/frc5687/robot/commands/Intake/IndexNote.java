package org.frc5687.robot.commands.Intake;

import org.frc5687.robot.Constants;
import org.frc5687.robot.OI;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.Intake;
import org.frc5687.robot.subsystems.Intake.IndexState;

public class IndexNote extends OutliersCommand {
    private final Intake _intake;
    private final OI _oi;

    public IndexNote(Intake intake, OI oi) {
        _intake = intake;
        _oi = oi;
        addRequirements(_intake);
    }

    @Override
    public void initialize() {
        super.initialize();
        _intake.setIndexState(IndexState.IDLE);
    }

    @Override
    public void execute() {
        switch (_intake.getIndexState()) {
            case IDLE:
                _intake.setSpeed(0);
                if (_oi.isIntakeButtonPressed()) {
                    _intake.setIndexState(IndexState.INTAKING);
                } else if (_intake.isNoteDetected()) {
                    _intake.setIndexState(IndexState.INDEXING);
                }
                break;

            case INTAKING:
                _intake.setSpeed(Constants.Intake.INTAKE_SPEED);
                if (_intake.isNoteDetected()) {
                    _intake.setIndexState(IndexState.INDEXING);
                } else if (!_oi.isIntakeButtonPressed()) {
                    _intake.setIndexState(IndexState.IDLE);
                }
                break;

            case INDEXING:
                if (_intake.isTopDetected() && _intake.isMiddleDetected() && _intake.isBottomDetected()) {
                    _intake.setSpeed(0);
                    _intake.setIndexState(IndexState.INDEXED);
                } else if (_intake.isMiddleDetected()) {
                    // Note is present in the robot
                    if (!_intake.isTopDetected() && !_intake.isBottomDetected()) {
                        // Note is moving away from the indexed position
                        _intake.setSpeed(Constants.Intake.INDEX_SPEED);
                    } else if (!_intake.isTopDetected() && _intake.isBottomDetected()) {
                        // This means we have overshot a bit.
                        _intake.setSpeed(Constants.Intake.REVERSE_INDEX_SPEED);
                    } else {
                        // Note is in the middle, maintain the current position
                        _intake.setSpeed(0);
                    }
                } else {
                    // Note is not detected by the middle sensor
                    _intake.setSpeed(Constants.Intake.INDEX_SPEED);
                }
                break;
            case INDEXED:
                _intake.setSpeed(0);
                if (_intake.isNoteDetected()) {
                    // Do nothing, note is already indexed
                } else if (_oi.isIntakeButtonPressed()) {
                    _intake.setIndexState(IndexState.INTAKING);
                } else {
                    _intake.setIndexState(IndexState.IDLE);
                }
                break;
            default:
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return false; // Always running
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        if (interrupted) {
            _intake.setSpeed(0);
            error("Index interrupted");
        }
        _intake.setSpeed(0);
        _intake.setIndexState(IndexState.IDLE);
    }
}