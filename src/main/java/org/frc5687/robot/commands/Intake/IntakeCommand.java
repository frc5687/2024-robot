package org.frc5687.robot.commands.Intake;

import org.frc5687.robot.Constants;
import org.frc5687.robot.OI;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.commands.RumbleGamepad;
import org.frc5687.robot.subsystems.Intake;

public class IntakeCommand extends OutliersCommand{
    private Intake _intake;
    private OI _oi;

    public IntakeCommand(Intake intake, OI oi) {
        _intake = intake;
        _oi = oi;
        addRequirements(_intake);
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        _intake.setSpeed(Constants.Intake.INTAKE_SPEED);
    }

    @Override
    public boolean isFinished() {
        return _intake.isBottomDetected();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        _intake.setSpeed(0);
        if (!interrupted) {
            new RumbleGamepad(_oi).schedule();
            new IndexNote(_intake).schedule();
        }
    }
}
