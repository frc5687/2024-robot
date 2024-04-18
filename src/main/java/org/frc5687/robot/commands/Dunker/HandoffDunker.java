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

    private double _rollerExtendRotation;
    private double _rollerDunkRotation;

    public HandoffDunker(Dunker dunker, Shooter shooter, Intake intake) {
        _dunker = dunker;
        _shooter = shooter;
        _intake = intake;
        addRequirements(dunker, shooter, intake);
    }

    @Override
    public void initialize() {
        super.initialize();
        _rollerDunkRotation = _dunker.getDunkerRollerRotations();
        _rollerExtendRotation = _dunker.getDunkerRollerRotations();
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
                boolean shooterAtTargetRPM = _shooter.isAtTargetRPM();
                boolean dunkerAtTargetRPM= _dunker.isAtTargetRPM();

                metric("shooter at rpm", shooterAtTargetRPM);
                metric("dunker at rpm", dunkerAtTargetRPM);

                if (shooterAtTargetRPM && dunkerAtTargetRPM) {
                    _intake.setSpeed(Constants.Intake.HANDOFF_SPEED);
                }
                if (_dunker.isNoteInDunker()) {
                    _shooter.setConfigSlot(1);
                    _shooter.setToStop();
                    _dunker.setToStop();
                    _intake.setSpeed(0);
                    _rollerDunkRotation = _dunker.getDunkerRollerRotations() + Constants.Dunker.ROLLER_RETRACT_ROTATIONS;
                    _rollerExtendRotation = _dunker.getDunkerRollerRotations() + Constants.Dunker.ROLLER_EXTEND_ROTATIONS;
                    // _dunker.setDunkerState(DunkerState.CLEAR_CAMERA_BAR);
                    _dunker.setDunkerState(DunkerState.NOTE_IN_DUNKER);
                }
                break;
            case CLEAR_CAMERA_BAR:
                // if (_dunker.getDunkerRollerRotations() > _rollerExtendRotation) {
                //     _dunker.setRollerSpeed(0);
                //     _dunker.setDunkerState(DunkerState.NOTE_IN_DUNKER);
                // } else {
                //     _dunker.setRollerSpeed(0.8);
                // }
                break;
            case NOTE_IN_DUNKER:
                if (_dunker.getDunkerRollerRotations() > _rollerExtendRotation) {
                    _dunker.setRollerSpeed(0);
                    // _dunker.setDunkerState(DunkerState.NOTE_IN_DUNKER);
                } else {
                    _dunker.setRollerSpeed(0.8);
                }
                _dunker.setDunkerAngle(Constants.Dunker.DUNK_ANGLE);

                if (_dunker.isAtAngle(Constants.Dunker.DUNK_ANGLE)) {
                    _dunker.setDunkerState(DunkerState.CORRECT_NOTE_LOCATION);
                }
                break;
            case CORRECT_NOTE_LOCATION:
                if (_dunker.getDunkerRollerRotations() < _rollerDunkRotation) {
                    _dunker.setRollerSpeed(0);
                    _dunker.setDunkerState(DunkerState.READY_TO_DUNK);
                } else {
                    _dunker.setRollerSpeed(-0.8);
                }
                break;  
            case READY_TO_DUNK:
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
