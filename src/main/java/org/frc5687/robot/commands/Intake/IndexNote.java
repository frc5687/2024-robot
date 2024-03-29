package org.frc5687.robot.commands.Intake;

import org.frc5687.robot.Constants;
import org.frc5687.robot.OI;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.commands.RumbleGamepad;
import org.frc5687.robot.subsystems.Intake;
import org.frc5687.robot.subsystems.Intake.IndexState;

import edu.wpi.first.wpilibj2.command.Command;

public class IndexNote extends OutliersCommand {
    private final Intake _intake;
    private final OI _oi;
    private Command _rumbleCommand;

    public IndexNote(Intake intake, OI oi) {
        _intake = intake;
        _oi = oi;
        _rumbleCommand = new RumbleGamepad(_oi);
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
                    _intake.setSpeed(0);
                    // error("Note detected");
                    _intake.setIndexState(IndexState.INDEXING);
                }
                break;

            case INTAKING:
                _intake.setSpeed(Constants.Intake.INTAKE_SPEED);
                if (_intake.isNoteDetected()) {
                    _intake.setSpeed(Constants.Intake.SLOW_INDEX_SPEED);
                    // error("Note detected moving to indexing");
                    _intake.setIndexState(IndexState.INDEXING);
                    _rumbleCommand.schedule();
                } else if (!_oi.isIntakeButtonPressed()) {
                    _intake.setSpeed(0);
                    // error("Intake button released going back to IDLE");
                    _intake.setIndexState(IndexState.IDLE);
                }
                break;

            case INDEXING:
                _oi.stopRumbleDriver();
                if (_intake.isTopDetected() && _intake.isMiddleDetected()) {
                    // error("Top and Middle detected, note should be indexed");
                    _intake.setSpeed(0);
                    _intake.setIndexState(IndexState.INDEXED);
                } else if (_intake.isMiddleDetected()) {
                    // error("Middle detected, slowing indexing");
                    _intake.setSpeed(Constants.Intake.SLOW_INDEX_SPEED);
                } else {
                    _intake.setSpeed(Constants.Intake.INTAKE_SPEED);
                }
                break;
            case INDEXED:
                _intake.setSpeed(0);
                if (_intake.isNoteDetected()) {
                    // Just to make sure.
                    _intake.setSpeed(0);
                    // Do nothing, note is already indexed
                } else {
                    // Just to make sure.
                    _intake.setSpeed(0);
                    _intake.setIndexState(IndexState.IDLE);
                }
                break;
            case SHOOTING:
                // This is when we exit out of a shot, should go back to idle
                if (_intake.isNoteDetected()) {
                    // Just dont shoot
                    _intake.setSpeed(0);
                    _intake.setIndexState(IndexState.INDEXED);
                } else {
                    _intake.setIndexState(IndexState.IDLE);
                }
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