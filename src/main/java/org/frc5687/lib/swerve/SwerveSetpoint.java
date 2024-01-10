package org.frc5687.lib.swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveSetpoint {
    public ChassisSpeeds chassisSpeeds;
    public SwerveModuleState[] moduleStates;

    public SwerveSetpoint(ChassisSpeeds chassisSpeeds, SwerveModuleState[] states) {
        this.chassisSpeeds = chassisSpeeds;
        this.moduleStates = states;
    }
}