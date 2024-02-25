package org.frc5687.robot.commands.Dunker;

import org.frc5687.robot.Constants;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.Dunker;
import org.frc5687.robot.subsystems.Dunker.DunkerState;

public class IdleDunker extends OutliersCommand{
    private Dunker _dunker;

    public IdleDunker(Dunker dunker) {
        _dunker = dunker;
        addRequirements(_dunker);
    }
    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        if (_dunker.getDunkerState() == DunkerState.UNKNOWN || _dunker.getDunkerState() == DunkerState.DUNKED_NOTE) {
            _dunker.setDunkerState(DunkerState.STOWING);
        }
        
        if (_dunker.getDunkerState() == DunkerState.STOWING) {
            _dunker.setDunkerAngle(Constants.Dunker.STOWED_ANGLE);
            if (_dunker.isAtAngle(Constants.Dunker.STOWED_ANGLE)) {
                _dunker.setDunkerState(DunkerState.STOWED);
                _dunker.disable(); // stop controlling the dunker, it rests on the hard stop to save energy
            }
        }
        // _dunker.setDunkerState(DunkerState.READY_TO_DUNK);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}


