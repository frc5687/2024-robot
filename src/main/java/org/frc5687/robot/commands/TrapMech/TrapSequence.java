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
    private long _timeout;
    private boolean _timeoutLock;
    private double _dunkerGoal;
    // slowly decrease the angle of the dunker to move arm slowly down during handoff.
    private double _slowAngleDunkerCreep;

    public TrapSequence(Dunker dunker, TrapMech trap, Climber climber, DriveTrain driveTrain) {
        _dunker = dunker;
        _trap = trap;
        _climber = climber;
        _driveTrain = driveTrain;
        addRequirements(_driveTrain, _trap, _climber, _dunker);
    }

    private void setTimeout(long timeout) {
        if (!_timeoutLock) {
            _timeout = System.currentTimeMillis() + timeout;
            error("Setting timeout to " + timeout);
            _timeoutLock = true;
        }
    }

    public void initialize() {
        _timeoutLock = false;
        _trap.setStep(TrapStep.DRIVING_TO_CHAIN);
        _slowAngleDunkerCreep = Constants.Dunker.TRAP_HANDOFF_ANGLE;
    }

    public void execute() {
        switch (_trap.getStep()) {
            case UNINITIALIZED:
                error("Setting to DRIVE_TO_CHAIN");
                _trap.setStep(TrapStep.DRIVING_TO_CHAIN);
                break;
            case DRIVING_TO_CHAIN:
                error("Setting to RELEASE_ELEVATOR");
                //pass for now, prolly gonna use a DriveToPose
                _timeoutLock = false;
                _trap.setStep(TrapStep.RELEASING_ELEVATOR);
                break;
            case RELEASING_ELEVATOR:
                _trap.releaseElevator();
                setTimeout(Constants.TrapMech.PNEUMATIC_TIMEOUT);
                if (System.currentTimeMillis() > _timeout) {
                    error("Setting to CLEAR_ARM");
                    _timeoutLock = false;
                    _trap.setStep(TrapStep.CLEARING_ARM);
                }
                break;
            case CLEARING_ARM:
                _trap.armUp();
                setTimeout(Constants.TrapMech.PNEUMATIC_TIMEOUT);
                if (System.currentTimeMillis() > _timeout) {
                    error("Setting to RETRACTING_NOTE");
                    _timeoutLock = false;
                    _trap.setStep(TrapStep.RETRACTING_NOTE);
                }
                break;
            case RETRACTING_NOTE:
                _dunkerGoal = _dunker.getRollerEncoder() - Constants.Dunker.RETRACT_DISTANCE;
                _dunker.setToRetractRPM();
                if (_dunker.getRollerEncoder() < _dunkerGoal) {
                    error("Setting to DUNKER_TO_TRAP");
                    _dunker.setToStop();
                    _trap.setStep(TrapStep.DUNKER_TO_TRAP_HANDOFF);
                }
                break;
            case DUNKER_TO_TRAP_HANDOFF:
                _dunker.setDunkerAngle(Constants.Dunker.TRAP_HANDOFF_ANGLE);
                if (_dunker.isAtAngle(Constants.Dunker.TRAP_HANDOFF_ANGLE)) {
                    error("Setting to ARM_TO_HANDOFF");
                    _trap.setStep(TrapStep.ARM_TO_HANDOFF);
                }
                break;
            case ARM_TO_HANDOFF:
                _trap.armDown();
                setTimeout(Constants.TrapMech.PNEUMATIC_TIMEOUT);
                if (System.currentTimeMillis() > _timeout && _dunker.isAtAngle(Constants.Dunker.TRAP_HANDOFF_ANGLE)) {
                    error("Setting to HANDOFF_TO_TRAP");
                    _timeoutLock = false;
                    _slowAngleDunkerCreep = Constants.Dunker.TRAP_HANDOFF_ANGLE;
                    _trap.setStep(TrapStep.HANDOFF_TO_TRAP);
                }
                break;
            case HANDOFF_TO_TRAP:
                _trap.setRollerSpeed(Constants.TrapMech.IN_SPEED);
                _dunker.setToHandoffRPM();
                _slowAngleDunkerCreep += Constants.Dunker.TRAP_CREEP_INCREMENT;
                _dunker.setDunkerAngle(_slowAngleDunkerCreep);
                setTimeout(Constants.TrapMech.HANDOFF_TIMEOUT);
                if (System.currentTimeMillis() > _timeout) {
                    error("Setting to WINCH_IN");
                    _trap.setRollerSpeed(0);
                    _dunker.setToStop();
                    _timeoutLock = false;
                    _trap.setStep(TrapStep.WINCH_IN);
                }
                break;
            case WINCH_IN:
                // _climber.setPositionMeters(Constants.Climber.TRAP_METERS);
                // if (Math.abs(_climber.getPositionMeters() - Constants.Climber.TRAP_METERS) < Constants.Climber.CLIMBER_TOLERANCE) {
                    _trap.setStep(TrapStep.ARM_TO_TRAP);
                // }
                break;
            case ARM_TO_TRAP:
                _trap.armUp();
                setTimeout(Constants.TrapMech.PNEUMATIC_TIMEOUT);
                if (System.currentTimeMillis() > _timeout) {
                    _timeoutLock = false;
                    _trap.setStep(TrapStep.SHOOT_TRAPMECH);
                }
                break;
            case SHOOT_TRAPMECH:
                _trap.setRollerSpeed(Constants.TrapMech.OUT_SPEED);
                setTimeout(Constants.TrapMech.SHOOT_TIMEOUT);
                if (System.currentTimeMillis() > _timeout) {
                    _timeoutLock = false;
                    _trap.setStep(TrapStep.DONE);
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