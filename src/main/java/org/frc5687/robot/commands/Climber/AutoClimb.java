package org.frc5687.robot.commands.Climber;

import org.frc5687.robot.subsystems.Climber;
import org.frc5687.robot.subsystems.DriveTrain;
import org.frc5687.robot.subsystems.Dunker;
import org.frc5687.robot.subsystems.Climber.ClimberStep;

import static org.frc5687.robot.Constants.DriveTrain.SLOW_KINEMATIC_LIMITS;

import org.frc5687.robot.Constants;
import org.frc5687.robot.OI;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.commands.Dunker.PositionForClimb;
import org.frc5687.robot.commands.Dunker.PositionForTrap;

public class AutoClimb extends OutliersCommand{
    
    private Climber _climber;
    private Dunker _dunker;
    private DriveTrain _driveTrain;
    private OI _oi;
    private double _climbHeight;
    
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
                    _climbHeight= Constants.Climber.PREP_METERS;
                    _climber.setStep(ClimberStep.CLEARING_DUNKER);
                }else if (_oi.getSoloClimbButton()){
                    _climbHeight = Constants.Climber.SOLO_METERS;
                    _climber.setStep(ClimberStep.CLEARING_DUNKER);
                }
                break;
            case CLEARING_DUNKER:
                if (!(_dunker.getCurrentCommand() instanceof PositionForTrap)) {
                    new PositionForClimb(_dunker).schedule(); // doing it this way because requiring dunker in this command would not work
                }
                if (Math.abs(Constants.Dunker.CLIMB_ANGLE - _dunker.getDunkerAngle()) < Constants.Dunker.ANGLE_TOLERANCE) {
                    _climber.setStep(ClimberStep.RAISING);
                }
                break;
            case RAISING:
                _climber.setPositionMeters(_climbHeight);
                _driveTrain.setKinematicLimits(SLOW_KINEMATIC_LIMITS);
                if (Math.abs(_climber.getPositionMeters() - _climbHeight) < Constants.Climber.CLIMBER_TOLERANCE) {
                    _climber.setStep(ClimberStep.RAISED);
                }
                break;
            case RAISED:
                _driveTrain.setKinematicLimits(SLOW_KINEMATIC_LIMITS);
                if (_oi.getClimbButton()) {
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
