package org.frc5687.robot.commands.Intake;

import org.frc5687.robot.Constants;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.Intake;

public class TimedIntake extends OutliersCommand{
    private Intake _intake;
    private double _timeout;
    public TimedIntake(Intake intake, double timeout) {
        _intake = intake;
        _timeout = timeout;
        addRequirements(_intake);
    }

    @Override
    public void initialize() {
        _timeout += System.currentTimeMillis();
        _intake.setSpeed(Constants.Intake.INTAKE_SPEED);
    }

    @Override
    public boolean isFinished() {
        return System.currentTimeMillis() >= _timeout;
    }

    @Override
    public void end(boolean interrupted) {
        _intake.setSpeed(0.0);
    }
}
