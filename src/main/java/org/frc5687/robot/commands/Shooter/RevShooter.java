package org.frc5687.robot.commands.Shooter;

import org.frc5687.robot.RobotState;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.Shooter;

public class RevShooter extends OutliersCommand {
    private Shooter _shooter;
    private RobotState _robotState = RobotState.getInstance();
    private Double _rpmOverride;

    public RevShooter(Shooter shooter) {
        this(shooter, null);
    }

    public RevShooter(Shooter shooter, Double rpmOverride) {
        _shooter = shooter;
        _rpmOverride = rpmOverride;
        addRequirements(_shooter);
    }

    @Override
    public void initialize() {
        error("Init RevShooter");
        super.initialize();
    }

    public void execute() {
        if (_rpmOverride != null) {
            _shooter.setShooterMotorRPM(_rpmOverride);
        } else {
            double distance = _robotState.getDistanceAndAngleToSpeaker().getFirst();
            _shooter.setRPMFromDistance(distance);
        }
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

    public boolean isFinished() {
        return false;
    }
}