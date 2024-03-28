package org.frc5687.robot.commands.TrapMech;

import org.frc5687.robot.Constants;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.Climber;
import org.frc5687.robot.subsystems.DriveTrain;
import org.frc5687.robot.subsystems.Dunker;
import org.frc5687.robot.subsystems.TrapMech;
import org.frc5687.robot.subsystems.TrapMech.TrapStep;

public class TrapSequence extends OutliersCommand{
    private Dunker _dunker;
    private TrapMech _trap;
    private Climber _climber;
    private DriveTrain _driveTrain;
    private double _timeout;
    private boolean _timeoutLock;
    private double _dunkerGoal;

    public TrapSequence(Dunker dunker, TrapMech trap, Climber climber, DriveTrain driveTrain) {
        _dunker = dunker;
        _trap = trap;
        _climber = climber;
        _driveTrain = driveTrain;
    }

    private void setTimeout(double timeout) {
        if (!_timeoutLock) {
            _timeout = System.currentTimeMillis() + timeout;
            _timeoutLock = true;
        }
    }

    public void initialize() {
        _trap.setStep(TrapStep.DRIVING_TO_CHAIN);
    }

    public void execute() {
        switch (_trap.getStep()) {
            case UNINITIALIZED:
                _trap.setStep(TrapStep.DRIVING_TO_CHAIN);
                break;
            case DRIVING_TO_CHAIN:
                //pass for now, prolly gonna use a DriveToPose
                _trap.setStep(TrapStep.RELEASING_ELEVATOR);
                break;
            case RELEASING_ELEVATOR:
                _trap.releaseElevator();
                setTimeout(Constants.TrapMech.PNEUMATIC_TIMEOUT);
                if (System.currentTimeMillis() > _timeout) {
                    _trap.setStep(TrapStep.CLEARING_ARM);
                    _timeoutLock = false;
                }
                break;
            case CLEARING_ARM:
                _trap.armUp();
                setTimeout(Constants.TrapMech.PNEUMATIC_TIMEOUT);
                if (System.currentTimeMillis() > _timeout) {
                    _trap.setStep(TrapStep.RETRACTING_NOTE);
                    _timeoutLock = false;
                }
                break;
            case RETRACTING_NOTE:
                _dunkerGoal = _dunker.getRollerEncoder() - Constants.Dunker.RETRACT_DISTANCE;
                _dunker.setToRetractRPM();
                if (_dunker.getRollerEncoder() < _dunkerGoal) {
                    _dunker.setToStop();
                    _trap.setStep(TrapStep.ARM_TO_HANDOFF);
                }
                break;
            case ARM_TO_HANDOFF:
                _trap.armDown();
                setTimeout(Constants.TrapMech.PNEUMATIC_TIMEOUT);
                if (System.currentTimeMillis() > _timeout) {
                    _trap.setStep(TrapStep.HANDOFF_TO_TRAP);
                    _timeoutLock = false;
                }
                break;
            case HANDOFF_TO_TRAP:
                _trap.setRollerSpeed(Constants.TrapMech.IN_SPEED);
                _dunker.setToHandoffRPM();
                if (_trap.isNoteInTrap()) {
                    _trap.setRollerSpeed(0);
                    _dunker.setToStop();
                    _trap.setStep(TrapStep.WINCH_IN);
                }
                break;
            case WINCH_IN:
                _climber.setPositionMeters(Constants.Climber.TRAP_METERS);
                if (Math.abs(_climber.getPositionMeters() - Constants.Climber.TRAP_METERS) < Constants.Climber.CLIMBER_TOLERANCE) {
                    _trap.setStep(TrapStep.ARM_TO_TRAP);
                }
                break;
            case ARM_TO_TRAP:
                _trap.armUp();
                setTimeout(Constants.TrapMech.PNEUMATIC_TIMEOUT);
                if (System.currentTimeMillis() > _timeout) {
                    _trap.setStep(TrapStep.SHOOT_TRAPMECH);
                    _timeoutLock = false;
                }
                break;
            case SHOOT_TRAPMECH:
                _trap.setRollerSpeed(Constants.TrapMech.OUT_SPEED);
                setTimeout(Constants.TrapMech.SHOOT_TIMEOUT);
                if (System.currentTimeMillis() > _timeout) {
                    _trap.setStep(TrapStep.DONE);
                    _timeoutLock = false;
                }
                break;
            case DONE:
                // all done :)
                break;
        }
    }

    public boolean isFinished() {
        return _trap.getStep() == TrapStep.DONE;
    }
}