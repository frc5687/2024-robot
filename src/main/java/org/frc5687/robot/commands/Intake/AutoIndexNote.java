package org.frc5687.robot.commands.Intake;

import org.frc5687.robot.Constants;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.Intake;
import org.frc5687.robot.subsystems.Intake.IndexState;

public class AutoIndexNote extends OutliersCommand {
    private final Intake _intake;

    public AutoIndexNote(Intake intake) {
        _intake = intake;
        addRequirements(_intake);
    }

    @Override
    public void initialize() {
        super.initialize();
        _intake.setIndexState(IndexState.INTAKING);
    }

    @Override
    public void execute() {
        switch (_intake.getIndexState()) {
            case IDLE:
                // this should never be called in here
                break;
            case INTAKING:
                _intake.setSpeed(Constants.Intake.INTAKE_SPEED);
                if (_intake.isNoteDetected()) {
                    _intake.setIndexState(IndexState.INDEXING);
                }
                break;

            case INDEXING:
                if (_intake.isTopDetected() && !_intake.isBottomDetected()) {
                    if (_intake.isTopDetected() && _intake.isMiddleDetected()) {
                        _intake.setSpeed(0);
                        _intake.setIndexState(IndexState.INDEXED);
                    } else {
                        _intake.setSpeed(Constants.Intake.REVERSE_INDEX_SPEED);
                    }
                } else if (!_intake.isTopDetected() && _intake.isBottomDetected()) {
                    _intake.setSpeed(Constants.Intake.INDEX_SPEED);
                } else if (_intake.isTopDetected() && _intake.isBottomDetected()) {
                    _intake.setSpeed(0);
                    _intake.setIndexState(IndexState.INDEXED);
                } else {
                    _intake.setSpeed(Constants.Intake.INDEX_SPEED);
                }
                break;

            case INDEXED:
                _intake.setSpeed(0);
                if (_intake.isNoteDetected()) {
                    _intake.setIndexState(IndexState.INDEXING);
                } else {
                    _intake.setIndexState(IndexState.INTAKING);
                }
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
    }


}