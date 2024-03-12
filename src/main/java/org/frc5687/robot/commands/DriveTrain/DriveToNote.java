package org.frc5687.robot.commands.DriveTrain;

import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotState;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.DriveTrain;
import org.frc5687.robot.subsystems.Intake;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

import java.util.Optional;

public class DriveToNote extends OutliersCommand {
    private final DriveTrain _driveTrain;
    private final ProfiledPIDController _xController;
    private final ProfiledPIDController _yController;
    private final ProfiledPIDController _yawController;
    private final RobotState _robotState = RobotState.getInstance();
    private final Intake _intake;

    public DriveToNote(DriveTrain driveTrain,  Intake intake) {
        _driveTrain = driveTrain;
        _xController = new ProfiledPIDController(2.0, 0.0, 0.0,
                new Constraints(Constants.DriveTrain.SLOW_KINEMATIC_LIMITS.maxDriveVelocity,
                        Constants.DriveTrain.SLOW_KINEMATIC_LIMITS.maxDriveAcceleration));
        _yController = new ProfiledPIDController(2.0, 0.0, 0.0,
                new Constraints(Constants.DriveTrain.SLOW_KINEMATIC_LIMITS.maxDriveVelocity,
                        Constants.DriveTrain.SLOW_KINEMATIC_LIMITS.maxDriveAcceleration));
        _yawController = new ProfiledPIDController(4.0, 0.0, 0.0,
                new Constraints(Constants.DriveTrain.MAX_ANG_VEL, Constants.DriveTrain.MAX_ANG_VEL * 4.0));
        _intake = intake;
        addRequirements(_driveTrain);
    }

    @Override
    public void initialize() {
        super.initialize();
        _xController.setGoal(0.0);
        _yController.setGoal(0.0);
        _yawController.setGoal(0.0);
    }

    @Override
    public void execute() {
        double vx = 0;
        double vy = 0;
        double rot = 0;

        if (_intake.isBottomDetected() || _intake.isTopDetected()) {
            return;
        }

        if (!_driveTrain.isLowGear()) {
            _driveTrain.shiftDownModules();
        }

        Optional<Pose3d> closestNoteRelativeField = _robotState.getClosestNoteRelativeRobotCenter();

        if (closestNoteRelativeField.isPresent()) {
            Pose2d notePose = closestNoteRelativeField.get().toPose2d();
            double x = notePose.getX();
            double y = notePose.getY();

            vx = -_xController.calculate(x);
            vy = -_yController.calculate(y);
            double angleToNoteRadians = Math.atan2(y, x);
            rot = -_yawController.calculate(angleToNoteRadians);

            _driveTrain.setVelocity(new ChassisSpeeds(vx, vy, rot));
        } else {
            _driveTrain.setVelocity(new ChassisSpeeds(0, 0, 0));
        }
    }

    @Override
    public boolean isFinished() {
        return false; // Determine your stop condition here
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}