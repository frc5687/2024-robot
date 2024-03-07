package org.frc5687.robot.commands.Shooter;

import java.util.Optional;

import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotState;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.Dunker;
import org.frc5687.robot.subsystems.Intake;
import org.frc5687.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

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
