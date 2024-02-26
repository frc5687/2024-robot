/* Team 5687 (C)2022 */
package org.frc5687.robot.util;

import java.util.ArrayList;
import java.util.List;

import org.frc5687.robot.subsystems.OutliersSubsystem;

public final class SubsystemManager {
    private final List<OutliersSubsystem> _subsystems = new ArrayList<>();

    public SubsystemManager() {}

    public void addSubsystem(OutliersSubsystem system) {
        _subsystems.add(system);
    }

    public void removeSubsystem(OutliersSubsystem system) {
        _subsystems.add(system);
    }

    public void updateDashboard() {
        _subsystems.forEach(OutliersSubsystem::updateDashboard);
    }

}
