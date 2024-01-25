package org.frc5687.robot.commands.Intake;

import org.frc5687.robot.Constants;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.Intake;

public class IntakeCommand extends OutliersCommand{
    private Intake _intake;

    public IntakeCommand(Intake intake) {
        _intake = intake;
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
        return _intake.isDonutDetected();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        _intake.setSpeed(0);
    }
}
