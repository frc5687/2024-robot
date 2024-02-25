package org.frc5687.robot.commands.Shooter;

import org.frc5687.robot.RobotState;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.Dunker;
import org.frc5687.robot.subsystems.Shooter;

public class IdleShooter extends OutliersCommand{
    private Shooter _shooter;
    private Dunker _dunker;
    private RobotState _robotState = RobotState.getInstance();

    public IdleShooter(Shooter shooter, Dunker dunker) {
        _shooter = shooter;
        _dunker = dunker;
        addRequirements(_shooter);
    }

    public void execute() {
        if (_dunker.getDunkerState() == Dunker.DunkerState.READY_TO_DUNK) {
            _shooter.setToStop(); // keep the note in while ready to dunk
        } else {
            _shooter.setToIdle();
        }
    }

    public boolean isFinished() {
        return false;
    }
}
