package org.frc5687.robot.commands.Climber;

import org.frc5687.robot.subsystems.Climber;
import org.frc5687.robot.subsystems.Climber.ClimberStep;
import org.frc5687.robot.Constants;
import org.frc5687.robot.OI;
import org.frc5687.robot.commands.OutliersCommand;

public class AutoClimb extends OutliersCommand{
    
    private Climber _climber;
    private OI _oi;
    
    public AutoClimb(Climber climber, OI oi) {
        _climber = climber;
        _oi = oi;
        addRequirements(_climber);
    }

    @Override
    public void execute() {
        switch (_climber.getStep()) {
            case UNKNOWN:
                break;
            case STOWED:
                if (_oi.getClimbButton()) {
                    _climber.setStep(ClimberStep.RAISING);
                }
                break;
            case RAISING:
                _climber.setPositionMeters(Constants.Climber.PREP_METERS);
                if (Math.abs(_climber.getPositionMeters() - Constants.Climber.PREP_METERS) < Constants.Climber.CLIMBER_TOLERANCE) {
                    _climber.setStep(ClimberStep.RAISED);
                }
                break;
            case RAISED:
                if (_oi.getClimbButton()) {
                    _climber.setStep(ClimberStep.LOWERING);
                }
                if (_oi.getStowButton()) {
                    _climber.setStep(ClimberStep.STOWING);
                }
                break;
            case LOWERING:
                _climber.setPositionMeters(Constants.Climber.CLIMB_METERS);
                if (Math.abs(_climber.getPositionMeters() - Constants.Climber.CLIMB_METERS) < Constants.Climber.CLIMBER_TOLERANCE) {
                    _climber.setStep(ClimberStep.LOWERED);
                }
                break;
            case LOWERED:
                // maybe add "press a button to go back to RAISING here?"
                if (_oi.getStowButton()) {
                    _climber.setStep(ClimberStep.STOWING);
                }
                break;
            case STOWING:
                _climber.setPositionMeters(Constants.Climber.LOWER_LIMIT);
                if (Math.abs(_climber.getPositionMeters() - Constants.Climber.LOWER_LIMIT) < Constants.Climber.CLIMBER_TOLERANCE) {
                    _climber.setStep(ClimberStep.STOWED);
                }
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
