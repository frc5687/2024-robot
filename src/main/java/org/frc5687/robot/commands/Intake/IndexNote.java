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
                if (_oi.isIntakeButtonPressed()) {
                    _intake.setSpeed(Constants.Intake.INTAKE_SPEED);
                    _intake.setIndexState(IndexState.INTAKING);
                } else {
                    _intake.setSpeed(0);
                }
                break;

            case INTAKING:
                if (_intake.isNoteDetected()) {
                    // we have seen the note on the bottom one (using the others as well for redundancy) - xavier bradford 4/4/24
                    _intake.setSpeed(Constants.Intake.INDEX_SPEED);
                    _intake.setIndexState(IndexState.BOTTOM_HAS_BEEN_DETECTED);
                } else if (!_oi.isIntakeButtonPressed()) {
                    // button released, stop
                    _intake.setSpeed(0);
                    _intake.setIndexState(IndexState.IDLE);
                } else {
                    // still intaking
                    _intake.setSpeed(Constants.Intake.INTAKE_SPEED);
                }
                break;

            case BOTTOM_HAS_BEEN_DETECTED:
                if (_intake.isMiddleDetected()) {
                    _intake.setSpeed(Constants.Intake.SLOW_INDEX_SPEED);
                    _intake.setIndexState(IndexState.MIDDLE_HAS_BEEN_DETECTED);
                    _rumbleCommand.schedule();
                }
                break;

            case MIDDLE_HAS_BEEN_DETECTED:
                if (_intake.isTopDetected()) {
                    _intake.setSpeed(0);
                    _intake.setIndexState(IndexState.INDEXED);
                }
                break;

            case INDEXED:
                _intake.setSpeed(0);
                _oi.stopRumbleDriver();
                if (_intake.isNoteDetected()) {
                    // Just to make sure.
                    _intake.setSpeed(0);
                    // Do nothing, note is already indexed
                } else {
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