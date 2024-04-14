package org.frc5687.robot.commands.DriveTrain;

import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotState;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.DriveTrain;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

import java.util.Optional;

public class DriveToNoteStopNoIntake extends OutliersCommand {
    private final DriveTrain _driveTrain;
    private final ProfiledPIDController _xController;
    private final ProfiledPIDController _yController;
    private final ProfiledPIDController _yawController;
    private final RobotState _robotState = RobotState.getInstance();
    private Optional<Pose3d> _closestNote = Optional.empty();
    private Optional<Long> _timeOut = Optional.empty();
    private boolean isFinished = false;

    public DriveToNoteStopNoIntake(DriveTrain driveTrain) {
        _driveTrain = driveTrain;
        _xController = new ProfiledPIDController(2.5, 0.0, 0.0,
                new Constraints(Constants.DriveTrain.SLOW_KINEMATIC_LIMITS.maxDriveVelocity,
                        Constants.DriveTrain.SLOW_KINEMATIC_LIMITS.maxDriveAcceleration));
        _yController = new ProfiledPIDController(2.5, 0.0, 0.0,
                new Constraints(Constants.DriveTrain.SLOW_KINEMATIC_LIMITS.maxDriveVelocity,
                        Constants.DriveTrain.SLOW_KINEMATIC_LIMITS.maxDriveAcceleration));
        _yawController = new ProfiledPIDController(4.0, 0.0, 0.0,
                new Constraints(Constants.DriveTrain.MAX_ANG_VEL, Constants.DriveTrain.MAX_ANG_VEL * 4.0));
        addRequirements(_driveTrain);
    }

    @Override
    public void initialize() {
        super.initialize();
        isFinished = false;
        _xController.setGoal(0.0);
        _yController.setGoal(0.0);
        _yawController.setGoal(0.0);
        _timeOut = Optional.empty();
        _closestNote = Optional.empty();
    }

    @Override
    public void execute() {
        double vx = 0;
        double vy = 0;
        double rot = 0;

        if (!_driveTrain.isLowGear()) {
            _driveTrain.shiftDownModules();
        }

        _closestNote = _robotState.getClosestNoteRelativeRobotCenter();

        if (_closestNote.isPresent()) {
            Pose2d notePose = _closestNote.get().toPose2d();
            double x = notePose.getX();
            double y = notePose.getY();

            vx = -_xController.calculate(x);
            vy = -_yController.calculate(y);
            double angleToNoteRadians = Math.atan2(y, x);
            rot = -_yawController.calculate(angleToNoteRadians);

            _driveTrain.setVelocity(new ChassisSpeeds(vx, vy, rot));
        } else {
            if (_timeOut.isEmpty()) {
                _timeOut = Optional.of(System.currentTimeMillis() + 250);
            } 

            if (_timeOut.isPresent()) {
                if (_timeOut.get() < System.currentTimeMillis()) {
                   isFinished = true;
                } else {
                   isFinished = false; 
                }
            }
            _driveTrain.setVelocity(new ChassisSpeeds(0, 0, 0));
        }
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}