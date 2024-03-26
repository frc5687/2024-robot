package org.frc5687.robot.commands.Shooter;

import org.frc5687.robot.Constants;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.Shooter;

import org.frc5687.robot.subsystems.Intake.IndexState;
import org.frc5687.robot.subsystems.Lights;
import org.frc5687.robot.subsystems.Intake;
import org.frc5687.robot.subsystems.DriveTrain;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class Pass extends OutliersCommand{
    private Shooter _shooter;
    private Intake _intake;
    private DriveTrain _driveTrain;
    private Lights _lights;

    public Pass(
        Shooter shooter,
        Intake intake,
        DriveTrain drivetrain,
        Lights lights
    ) {
        _shooter = shooter;
        _intake = intake;
        _driveTrain = drivetrain;
        _lights = lights;
        addRequirements(_shooter, _intake,_driveTrain,_lights);
    }


    @Override
    public void initialize() {
        super.initialize();
        _intake.setIndexState(IndexState.SHOOTING);
    }
    @Override
    public void execute() {
        _shooter.setToPassRPM();

        ChassisSpeeds speeds = _driveTrain.getMeasuredChassisSpeeds();
        boolean isStopped = (speeds.vxMetersPerSecond < 0.1 && speeds.vyMetersPerSecond < 0.1);

        boolean isAtTargetRPM = _shooter.isAtTargetRPM();
       
        if (isAtTargetRPM && isStopped) {
            _intake.setSpeed(Constants.Intake.INTAKE_SPEED);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        _intake.setIndexState(IndexState.IDLE);
        // _driveTrain.setMaintainHeading(_driveTrain.getHeading());
    }
}
