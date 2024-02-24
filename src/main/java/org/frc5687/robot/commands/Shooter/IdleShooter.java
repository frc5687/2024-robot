package org.frc5687.robot.commands.Shooter;

import org.frc5687.robot.RobotState;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.Shooter;

public class IdleShooter extends OutliersCommand{
    private Shooter _shooter;
    private RobotState _robotState = RobotState.getInstance();

    public IdleShooter(Shooter shooter) {
        _shooter = shooter;
        addRequirements(_shooter);
    }

    public void execute() {
        _shooter.setTargetRPM(_shooter.calculateRPMFromDistance(_robotState.getDistanceAndAngleToSpeaker().getFirst()));
        _shooter.setToTarget();
    }

    public boolean isFinished() {
        return false;
    }
}
