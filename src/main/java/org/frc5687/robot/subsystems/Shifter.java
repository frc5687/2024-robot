package org.frc5687.robot.subsystems;

import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotMap;
import org.frc5687.robot.util.OutliersContainer;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Shifter extends OutliersSubsystem {
    private final DoubleSolenoid _shift;
    private final Compressor _compressor;
    private final DriveTrain _driveTrain;

    private boolean _isLowGear;
    // TODO shift the drive train to low gear at 40psi -- ask simeon about this

    public Shifter (OutliersContainer container, DriveTrain driveTrain) {
        super(container);
        _driveTrain = driveTrain;
        _shift = new DoubleSolenoid(
                PneumaticsModuleType.REVPH,
                RobotMap.PCM.SHIFTER_HIGH,
                RobotMap.PCM.SHIFTER_LOW);
        // create compressor, compressor logic
        _compressor = new Compressor(PneumaticsModuleType.REVPH);
        _compressor.enableAnalog(Constants.DriveTrain.MIN_PSI, Constants.DriveTrain.MAX_PSI);
    }

    @Override
    public void updateDashboard() {
        metric("Tank Pressure PSI", _compressor.getPressure());
        metric("Is Low Gear", _isLowGear);
        metric("Current Command", getCurrentCommand() != null ? getCurrentCommand().getName() : "no command");
    }

    public void shiftUp() {
        _shift.set(Value.kForward);
        _isLowGear = false;
        // this is the only correct use of the shiftUpModules method - xavier bradford 03/02/24
        _driveTrain.shiftUpModules();
    }

    public void shiftDown() {
        _shift.set(Value.kReverse);
        _isLowGear = true;
        // this is the only correct use of the shiftUpModules method - xavier bradford 03/02/24
        _driveTrain.shiftDownModules();
    }

    public boolean isLowGear() {
        return _isLowGear;
    }
}
