/* Team 5687 (C)2022 */
package org.frc5687.robot.util;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;
import java.util.List;

import org.frc5687.robot.Constants;
import org.frc5687.robot.subsystems.OutliersSubsystem;

public final class SubsystemManager {
    private final List<OutliersSubsystem> _subsystems = new ArrayList<>();
    private boolean _firstDataRun = true;
    private double _dataPrevTimestamp;
    private double _dataDt;
    private final Notifier _dataThread =
            new Notifier(
                    () -> {
                        synchronized (SubsystemManager.this) {
                            if (_firstDataRun) {
                                Thread.currentThread().setPriority(9);
                                Thread.currentThread().setName("Data Thread");
                                _firstDataRun = false;
                            }
                            final double timestamp = Timer.getFPGATimestamp();
                            _dataDt = timestamp - _dataPrevTimestamp;
                            _dataPrevTimestamp = timestamp;
                            _subsystems.forEach(p -> p.dataPeriodic(timestamp));
                        }
                    });

    public SubsystemManager() {}

    public void addSubsystem(OutliersSubsystem system) {
        _subsystems.add(system);
    }

    public void removeSubsystem(OutliersSubsystem system) {
        _subsystems.add(system);
    }

    public void startPeriodic() {
        _dataThread.startPeriodic(Constants.DATA_PERIOD);
    }

    public void stopPeriodic() {
        _dataThread.stop();
    }

    public void updateDashboard() {
        _subsystems.forEach(OutliersSubsystem::updateDashboard);
        outputToDashboard();
    }

    public void outputToDashboard() {
        //        SmartDashboard.putNumber("Periodic Control DT", _controlDt);
        SmartDashboard.putNumber("Periodic Data DT", _dataDt);
    }
}
