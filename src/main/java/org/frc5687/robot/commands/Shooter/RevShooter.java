package org.frc5687.robot.commands.Shooter;

import org.frc5687.robot.RobotState;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.Shooter;

public class RevShooter extends OutliersCommand{
    private Shooter _shooter;
    private RobotState _robotState = RobotState.getInstance();

    public RevShooter(Shooter shooter) {
        _shooter = shooter;
        addRequirements(_shooter);
    }
    @Override
    public void initialize() {
        super.initialize();
    }
    public void execute() {
        // FIXME this might be broken on red or blue alliance in autos. it might not realize it's on the correct alliance and therefore will not rev, leading to a difficult to debug problem where autos run slower on one side than the other. - xavier bradford 03/09/24
        double distance = _robotState.getDistanceAndAngleToSpeaker().getFirst();
        _shooter.setRPMFromDistance(distance);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

    public boolean isFinished() {
        return false;
    }
}
