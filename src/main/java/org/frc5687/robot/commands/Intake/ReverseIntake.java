package org.frc5687.robot.commands.Intake;

import org.frc5687.robot.Constants;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.Intake;

public class ReverseIntake extends OutliersCommand{
    private Intake _intake;

    public ReverseIntake(Intake intake) {
        _intake = intake;
        addRequirements(_intake);
    }

    public void execute() {
        _intake.setSpeed(Constants.Intake.REVERSE_INTAKE_SPEED);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
