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
        error("Init AutoIntake");
        _intake.setIndexState(IndexState.INTAKING);
    }

    @Override
    public void execute() {
        switch (_intake.getIndexState()) {
            case INTAKING:
                if (_intake.isNoteDetected()) {
                    _intake.setSpeed(Constants.Intake.INDEX_SPEED);
                    _intake.setIndexState(IndexState.BOTTOM_HAS_BEEN_DETECTED);
                } else {
                    // no note, intake hard
                    _intake.setSpeed(Constants.Intake.INTAKE_SPEED);
                }
                break;
            case BOTTOM_HAS_BEEN_DETECTED:
                if (_intake.isTopDetected()) {
                    _intake.setSpeed(0);
                    _intake.setIndexState(IndexState.INDEXED);
                } else if (_intake.isMiddleDetected()) {
                    _intake.setSpeed(Constants.Intake.SLOW_INDEX_SPEED);
                    _intake.setIndexState(IndexState.MIDDLE_HAS_BEEN_DETECTED);
                }
                break;
            case MIDDLE_HAS_BEEN_DETECTED:
                if (_intake.isTopDetected()) {
                    _intake.setSpeed(0);
                    _intake.setIndexState(IndexState.INDEXED);
                }
                break;
            default:
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return _intake.getIndexState() == IndexState.INDEXED;
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