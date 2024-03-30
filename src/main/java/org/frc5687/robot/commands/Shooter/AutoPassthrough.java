package org.frc5687.robot.commands.Shooter;

import org.frc5687.robot.subsystems.Intake;
import org.frc5687.robot.subsystems.Shooter;
import org.frc5687.robot.Constants;
import org.frc5687.robot.commands.OutliersCommand;

public class AutoPassthrough extends OutliersCommand {

    private Shooter _shooter;
    private Intake _intake;
    private long _timeOut;
    private long _timeOutDuration;

    public AutoPassthrough(Shooter shooter, Intake intake, long timeOutDuration) {
        _shooter = shooter;
        _intake = intake;
        _timeOutDuration = timeOutDuration;
        addRequirements(_shooter, _intake);
    }

    @Override
    public void initialize() {
        _timeOut = System.currentTimeMillis() + _timeOutDuration;
    }

    @Override
    public void execute() {
        _shooter.setToPassthrough();
        _intake.setSpeed(Constants.Intake.INTAKE_SPEED);
    }

    @Override
    public void end(boolean interrupted) { // not sure if this matters, idle commands might change it immediately.
        _shooter.setToIdle();
        _intake.setSpeed(0);
    }

    @Override
    public boolean isFinished() {
        if (System.currentTimeMillis() > _timeOut){
            return true;
        } else {
            return false;
        }
    }
}