package org.frc5687.robot.subsystems;

import org.frc5687.robot.RobotContainer;
import org.frc5687.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class TrapMech extends OutliersSubsystem {
    private final TalonSRX _roller;
    //FIXME No idea if this forward/reverse configuration is correct
    private final DoubleSolenoid _release;
    private final DoubleSolenoid _arm;

    public TrapMech(RobotContainer container){
        super(container);
        _roller = new TalonSRX(RobotMap.CAN.TALONSRX.TRAPMECH_ROLLER);
        _release = new DoubleSolenoid(PneumaticsModuleType.REVPH, RobotMap.PCM.RELEASE_ATTACH, RobotMap.PCM.RELEASE_DETACH);
        _arm = new DoubleSolenoid(PneumaticsModuleType.REVPH, RobotMap.PCM.ARM_RAISE, RobotMap.PCM.ARM_LOWER);
    }

    public void setRollerSpeed(double value) {
        _roller.set(TalonSRXControlMode.PercentOutput, value);
    }

    public void releaseElevator() {
        _release.set(Value.kReverse);
    }

    public void armUp() {
        _arm.set(Value.kForward);
    }

    public void armDown() {
        _arm.set(Value.kReverse);
    }

    public void updateDashboard() {}

}
