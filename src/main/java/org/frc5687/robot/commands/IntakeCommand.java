package org.frc5687.robot.commands;

import org.frc5687.robot.Constants;
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
        _intake.setSpeed(Constants.INTAKE.INTAKE_SPEED);
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return false; 
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        _intake.setSpeed(0);
    }
}
