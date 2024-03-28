package org.frc5687.robot.commands.Shooter;

import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.Shooter;

public class ToggleAutoSpinUp extends OutliersCommand{
    public Shooter _shooter;

    public ToggleAutoSpinUp(Shooter shooter) {
        _shooter = shooter;
    }

    @Override
    public void initialize() {
        if (_shooter.getSpinUpAutomatically()) {
            // start in speaker mode, go to amp mode
            _shooter.setSpinUpAutomatically(false);
            _shooter.setToHandoffRPM();
        } else {
            // start in amp mode, go to speaker mode
            _shooter.setSpinUpAutomatically(true);
            _shooter.setToIdle();
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}