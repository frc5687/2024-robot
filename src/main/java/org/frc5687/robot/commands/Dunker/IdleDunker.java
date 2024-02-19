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
        if (_dunker.getDunkerState() != DunkerState.UNKNOWN || _dunker.getDunkerState() != DunkerState.DUNKED_NOTE) {
            _dunker.setDunkerAngle(Constants.Dunker.STOWED_ANGLE);
            _dunker.setDunkerState(DunkerState.STOWED);
        }
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

