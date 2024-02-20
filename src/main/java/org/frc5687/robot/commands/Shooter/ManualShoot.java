package org.frc5687.robot.commands.Shooter;

import org.frc5687.lib.control.SwerveHeadingController.HeadingState;
import org.frc5687.robot.Constants;
import org.frc5687.robot.Robot;
import org.frc5687.robot.RobotState;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.Shooter;
import org.frc5687.robot.subsystems.Deflector;
import org.frc5687.robot.subsystems.DriveTrain;
import org.frc5687.robot.subsystems.Intake;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ManualShoot extends OutliersCommand{
    private Shooter _shooter;
    private Deflector _deflector;
    private Intake _intake;
    private RobotState _robotState = RobotState.getInstance();

    public ManualShoot(
        Shooter shooter,
        Deflector deflector,
        Intake intake
    ) {
        _shooter = shooter;
        _deflector = deflector;
        _intake = intake;
        addRequirements(_shooter, _intake, _deflector);
    }

    @Override
    public void execute() {
        Pair<Double, Double> distanceAndAngle = _robotState.getDistanceAndAngleToSpeaker();

        double distance = distanceAndAngle.getFirst();

        double angle = distanceAndAngle.getSecond();

            _shooter.setTargetRPM(2000);
            _shooter.setToTarget();
            _deflector.setTargetAngle(Constants.Deflector.IDLE_ANGLE);

        if (_shooter.isAtTargetRPM() /*&& _deflector.isAtTargetAngle()*/) { 
            _intake.setSpeed(Constants.Intake.INTAKE_SPEED);
        }

        SmartDashboard.putNumber("Angle to shoot", angle);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // _driveTrain.setMaintainHeading(_driveTrain.getHeading());
    }
}
