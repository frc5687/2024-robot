package org.frc5687.robot.commands.Dunker;

import org.frc5687.robot.Constants;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.Dunker;
import org.frc5687.robot.subsystems.Shooter;
import org.frc5687.robot.subsystems.Dunker.DunkerState;

public class DunkNote extends OutliersCommand {
    private Dunker _dunker;
    private Shooter _shooter;
    private long _ejectTime;

    public DunkNote(Dunker dunker, Shooter shooter) {
        _dunker = dunker;
        _shooter = shooter;
        addRequirements(_dunker, _shooter);
    }

    @Override
    public void initialize() {
        super.initialize();
        _shooter.setConfigSlot(0);
        // if (_dunker.getDunkerState() != DunkerState.READY_TO_DUNK) {
        //     error("Dunk note called without being ready!");
        //     //cancels the command, then runs end(). end() will set the state to dunkednote, and idledunker will stow it again.
        //     this.cancel();
        // }
        _ejectTime = System.currentTimeMillis() + Constants.Dunker.EJECT_TIME;
    }

    @Override
    public void execute() {
        super.execute();
        _shooter.setToDunkOutRPM();
    }

    @Override
    public boolean isFinished() {
        return (System.currentTimeMillis() > _ejectTime);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        _dunker.setDunkerState(DunkerState.DUNKED_NOTE);
    }
}
