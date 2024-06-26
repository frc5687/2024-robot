package org.frc5687.robot.commands.Shooter;

import org.frc5687.robot.RobotState;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.Intake;
import org.frc5687.robot.subsystems.Shooter;

public class IdleShooter extends OutliersCommand {
    private Shooter _shooter;
    private Intake _intake;
    private RobotState _robotState = RobotState.getInstance();

    public IdleShooter(Shooter shooter, Intake intake) {
        _shooter = shooter;
        _intake = intake;
        addRequirements(_shooter);
    }

    public void execute() {
        // if (_robotState.isWithinOptimalRange() && _intake.isNoteDetected() && !_shooter.getAmpMode()) {
        //     _shooter.setRPMFromDistance(_robotState.getDistanceAndAngleToSpeaker().getFirst());
        // } else {
            _shooter.setToIdle();
        // }

    }


    public boolean isFinished() {
        return false;
    }
}
