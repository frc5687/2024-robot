package org.frc5687.robot.commands.Dunker;

import org.frc5687.robot.Constants;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.Dunker;

public class PositionForClimb extends OutliersCommand {
    private Dunker _dunker;
    public PositionForClimb(Dunker dunker) {
        _dunker = dunker;
        addRequirements(dunker);
    }

    @Override
    public void execute() {
        _dunker.setDunkerAngle(Constants.Dunker.CLIMB_ANGLE);
        // if (_dunker.isAtAngle(Constants.Dunker.CLIMB_ANGLE)) {
        //     _dunker.disable();
        // }
    }

    @Override
    public boolean isFinished() {
        return false; // this keeps the dunker from going back to its original position. perhaps we should allow the dunker to go back in case of an accidental climb?
    }
}
