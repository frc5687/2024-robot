package org.frc5687.robot.commands.Climber;

import org.frc5687.robot.subsystems.Climber;
import org.frc5687.robot.subsystems.DriveTrain;
import org.frc5687.robot.subsystems.Dunker;
import org.frc5687.robot.subsystems.DriveTrain;
import org.frc5687.robot.subsystems.Climber.ClimberStep;

import static org.frc5687.robot.Constants.DriveTrain.SLOW_KINEMATIC_LIMITS;

import org.frc5687.robot.Constants;
import org.frc5687.robot.OI;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.commands.Dunker.PositionForClimb;

public class AutoClimb extends OutliersCommand{
    
    private Climber _climber;
    private Dunker _dunker;
    private DriveTrain _driveTrain;
    private OI _oi;
    
    public AutoClimb(Climber climber, Dunker dunker, DriveTrain driveTrain, OI oi) {
        _climber = climber;
        _dunker = dunker;
        _driveTrain = driveTrain;
        _oi = oi;
        addRequirements(_climber);
    }

    @Override
    public void execute() {
        switch (_climber.getStep()) {
            case STOWED:
                if (_oi.getClimbButton()) {
                    _climber.setStep(ClimberStep.CLEARING_DUNKER);
                }
                break;
            case CLEARING_DUNKER:
                if (!(_dunker.getCurrentCommand() instanceof PositionForClimb)) {
                    new PositionForClimb(_dunker).schedule(); // doing it this way because requiring dunker in this command would not work
                }
                if (_dunker.getDunkerAngle() < Constants.Dunker.PREP_ANGLE) {
                    _climber.setStep(ClimberStep.RAISING);
                }
                break;
            case RAISING:
                _climber.setPositionMeters(Constants.Climber.PREP_METERS);
                _driveTrain.setKinematicLimits(SLOW_KINEMATIC_LIMITS);
                if (Math.abs(_climber.getPositionMeters() - Constants.Climber.PREP_METERS) < Constants.Climber.CLIMBER_TOLERANCE) {
                    _climber.setStep(ClimberStep.RAISED);
                }
                break;
            case RAISED:
                _driveTrain.setKinematicLimits(SLOW_KINEMATIC_LIMITS);
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
