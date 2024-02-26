package org.frc5687.robot.commands.Dunker;

import org.frc5687.robot.Constants;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.Dunker;
import org.frc5687.robot.subsystems.Intake;
import org.frc5687.robot.subsystems.Shooter;
import org.frc5687.robot.subsystems.Dunker.DunkerState;

public class HandoffDunker extends OutliersCommand {
    private Dunker _dunker;
    private Shooter _shooter;
    private Intake _intake;

    public HandoffDunker(Dunker dunker, Shooter shooter, Intake intake) {
        _dunker = dunker;
        _shooter = shooter;
        _intake = intake;
        addRequirements(dunker, shooter, intake);
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        switch (_dunker.getDunkerState()) {
            case STOWING: // this case should be taken care of by IdleDunker, but just in case...
                _dunker.setDunkerState(DunkerState.STOWED);
                break;
            case STOWED:
                _dunker.setDunkerAngle(Constants.Dunker.PREP_ANGLE);
                if (_dunker.isAtAngle(Constants.Dunker.PREP_ANGLE)) {
                    _dunker.setDunkerState(DunkerState.PREPARED_FOR_NOTE);
                }
                break;
            case PREPARED_FOR_NOTE:
                _shooter.setToDunkInRPM();
                _intake.setSpeed(Constants.Intake.HANDOFF_SPEED);
                if (_dunker.isNoteInDunker()) {
                    _shooter.setToStop();
                    _intake.setSpeed(0);
                    _dunker.setDunkerState(DunkerState.NOTE_IN_DUNKER);
                }
                break;
            case NOTE_IN_DUNKER:
                _dunker.setDunkerAngle(Constants.Dunker.DUNK_ANGLE);
                if (_dunker.isAtAngle(Constants.Dunker.DUNK_ANGLE)) {
                    _dunker.setDunkerState(DunkerState.READY_TO_DUNK);
                }
                break;
            case READY_TO_DUNK:
                // finish the command, the dunker has the note and is READY :)
                break;
            case DUNKED_NOTE:
                // you should never be here in this command
                break;
            default:
                break;
            
        }

        super.execute();
    }

    @Override
    public boolean isFinished() {
        return _dunker.getDunkerState() == DunkerState.READY_TO_DUNK;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}

