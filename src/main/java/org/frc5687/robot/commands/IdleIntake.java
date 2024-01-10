package org.frc5687.robot.commands;

import org.frc5687.robot.Constants;
import org.frc5687.robot.subsystems.Intake;

public class IdleIntake extends OutliersCommand{
    private Intake _intake;

    public IdleIntake(Intake intake) {
        _intake = intake;
        addRequirements(_intake);
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        _intake.setSpeed(0);

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

