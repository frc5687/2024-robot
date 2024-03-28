package org.frc5687.robot.commands.Dunker;

import org.frc5687.robot.Constants;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.Dunker;
import org.frc5687.robot.subsystems.Intake;
import org.frc5687.robot.subsystems.Shooter;
import org.frc5687.robot.subsystems.Dunker.DunkerState;

public class SetupForTrap extends OutliersCommand {
    private Dunker _dunker;
    private Shooter _shooter;
    private Intake _intake;

    public SetupForTrap(Dunker dunker, Shooter shooter, Intake intake) {
        _dunker = dunker;
        _shooter = shooter;
        _intake = intake;
        addRequirements(dunker, shooter, intake);
    }

    @Override
    public void initialize() {
        super.initialize();
        //Pretend like the dunker was stowed, hopefully this takes care of any state the dunker may be in when we start trap seq (except ReadyToDunk :P)
        _dunker.setDunkerState(DunkerState.STOWED);
    }

    @Override
    public void execute() {
        switch (_dunker.getDunkerState()) {
            case STOWING: // this case should be taken care of by IdleDunker, but just in case...
                _dunker.setDunkerState(DunkerState.STOWED);
                break;
            case STOWED:
                _dunker.setToHandoffRPM();
                _shooter.setToHandoffRPM();
                _dunker.setDunkerAngle(Constants.Dunker.PREP_ANGLE);
                if (_dunker.isAtAngle(Constants.Dunker.PREP_ANGLE)) {
                    _dunker.setDunkerState(DunkerState.PREPARED_FOR_NOTE);
                }
                break;
            case PREPARED_FOR_NOTE:
                _dunker.setToHandoffRPM();
                _shooter.setToHandoffRPM();
                if (_dunker.isAtTargetRPM() && _shooter.isAtTargetRPM()) {
                    _intake.setSpeed(Constants.Intake.HANDOFF_SPEED);
                }
                if (_dunker.isNoteInDunker()) {
                    _shooter.setConfigSlot(1);
                    _shooter.setToStop();
                    _dunker.setToStop();
                    _intake.setSpeed(0);
                    _dunker.setDunkerState(DunkerState.NOTE_IN_DUNKER);
                }
                break;
            case NOTE_IN_DUNKER:
                // finish the command, the dunker has the note and is READY :)
                break;
            default:
                //god forbid this ever occurs
                break;

        }

        super.execute();
    }

    @Override
    public boolean isFinished() {
        return _dunker.getDunkerState() == DunkerState.NOTE_IN_DUNKER;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}
