package org.frc5687.robot.commands.Intake;

import org.frc5687.robot.Constants;
import org.frc5687.robot.OI;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.commands.RumbleGamepad;
import org.frc5687.robot.subsystems.Intake;

public class IntakeCommand extends OutliersCommand{
    private Intake _intake;
    private OI _oi;
    private boolean _noteInIndex;

    public IntakeCommand(Intake intake, OI oi) {
        _intake = intake;
        _oi = oi;
        _noteInIndex = false;
        addRequirements(_intake);
    }

    @Override
    public void initialize() {
        super.initialize();
        if (_intake.isNoteIndexed()) {
            _noteInIndex = true;
            error("Note already indexed!");
        }
    }

    @Override
    public void execute() {
        if (!_noteInIndex) {
            _intake.setSpeed(Constants.Intake.INTAKE_SPEED);
        } else {
            _intake.setSpeed(0);
        }
    }

    @Override
    public boolean isFinished() {
        return _intake.isBottomDetected() || _noteInIndex; // check periodicy as well incase bottom sensor misses the index
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        _intake.setSpeed(0);
        if (!interrupted) {
            _intake.setSpeed(0);
            new IndexNote(_intake).schedule();
            new RumbleGamepad(_oi).schedule();
        }
    }
}
