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

public class IdleShooter extends OutliersCommand{
    private Shooter _shooter;
    private Dunker _dunker;
    private Intake _intake;
    private RobotState _robotState = RobotState.getInstance();

    public IdleShooter(Shooter shooter, Dunker dunker, Intake intake) {
        _shooter = shooter;
        _dunker = dunker;
        _intake = intake;
        addRequirements(_shooter);
    }

    public void execute() {
        if (_dunker.getDunkerState() == Dunker.DunkerState.READY_TO_DUNK) {
            _shooter.setToStop(); // keep the note in while ready to dunk
        } else {
            boolean onSpeakerFieldHalf = false;
            Optional<Alliance> optionalAlliance = DriverStation.getAlliance();
            if (optionalAlliance.isPresent()) {
                Alliance alliance = optionalAlliance.get();
                if (alliance == Alliance.Blue && _robotState.getEstimatedPose().getX() < Constants.FieldConstants.FIELD_LENGTH / 2) {
                    onSpeakerFieldHalf = true;
                } else if (alliance == Alliance.Red && _robotState.getEstimatedPose().getX() > Constants.FieldConstants.FIELD_LENGTH / 2) {
                    onSpeakerFieldHalf = true;
                }
            }

            // spin up if on speaker half of the field and has note

            boolean hasNote = false;
            if (_intake.isBottomDetected() || _intake.isTopDetected()) {
                hasNote = true;
            }
            
            if (onSpeakerFieldHalf && hasNote) {
                _shooter.setTargetRPM(_shooter.calculateRPMFromDistance(_robotState.getDistanceAndAngleToSpeaker().getFirst()));
                _shooter.setToTarget();
            } else {
                _shooter.setToIdle();
            }
        }
    }

    public boolean isFinished() {
        return false;
    }
}
