package org.frc5687.robot.subsystems;

import org.frc5687.lib.sensors.ProximitySensor;
import org.frc5687.robot.RobotContainer;
import org.frc5687.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class TrapMech extends OutliersSubsystem {
    private final TalonSRX _roller;
    private final ProximitySensor _prox;
    //FIXME No idea if this forward/reverse configuration is correct
    private final DoubleSolenoid _release;
    private final DoubleSolenoid _arm;


    private TrapStep _step = TrapStep.UNINITIALIZED;

    public TrapMech(RobotContainer container){
        super(container);
        _roller = new TalonSRX(RobotMap.CAN.TALONSRX.TRAPMECH_ROLLER);
        _prox = new ProximitySensor(RobotMap.DIO.TRAP_PROXIMIMITY);
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

    public boolean isNoteInTrap() {
        return _prox.get();
    }

    public void updateDashboard() {}

    public TrapStep getStep() {
        return _step;
    }

    public void setStep(TrapStep step) {
        _step = step;
    }

    public enum TrapStep {
        UNINITIALIZED(0),
        DRIVING_TO_CHAIN(1),
        RELEASING_ELEVATOR(2),
        CLEARING_ARM(3),
        RETRACTING_NOTE(4),
        ARM_TO_HANDOFF(5),
        HANDOFF_TO_TRAP(6),
        WINCH_IN(7),
        ARM_TO_TRAP(8),
        SHOOT_TRAPMECH(9),
        DONE(10);

        private final int _value;
        TrapStep(int value) { 
            _value = value; 
        }

        public int getValue() { 
            return _value; 
        }
    }

}
