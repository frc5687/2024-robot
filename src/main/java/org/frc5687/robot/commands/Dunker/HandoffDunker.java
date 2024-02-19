package org.frc5687.robot.commands.Dunker;

import org.frc5687.robot.Constants;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.Dunker;
import org.frc5687.robot.subsystems.Shooter;
import org.frc5687.robot.subsystems.Dunker.DunkerState;

public class HandoffDunker extends OutliersCommand {
    private Dunker _dunker;
    private Shooter _shooter;

    public HandoffDunker(Dunker dunker, Shooter shooter) {
        _dunker = dunker;
        _shooter = shooter;
        addRequirements(_dunker, shooter);
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        switch (_dunker.getDunkerState()) {
            case STOWED:
                _dunker.setDunkerAngle(Constants.Dunker.PREP_ANGLE);
                if (_dunker.isAtAngle(Constants.Dunker.PREP_ANGLE)) {
                    _dunker.setDunkerState(DunkerState.PREPARED_FOR_NOTE);
                }
                break;
            case PREPARED_FOR_NOTE:
                _shooter.setTargetRPM(300);
                _shooter.setToTarget();
                if (_dunker.isNoteInDunker()) {
                    _shooter.setTargetRPM(0);
                    _shooter.setToTarget();
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

