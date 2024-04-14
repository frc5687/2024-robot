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

public class DriveToNoteStop extends OutliersCommand {
    private final DriveTrain _driveTrain;
    private final ProfiledPIDController _xController;
    private final ProfiledPIDController _yController;
    private final ProfiledPIDController _yawController;
    private final RobotState _robotState = RobotState.getInstance();
    private final Intake _intake;
    private Optional<Pose3d> _closestNote = Optional.empty();
    private IndexState _state;
    private Optional<Long> _timeOut = Optional.empty();
    private boolean isFinished = false;

    public DriveToNoteStop(DriveTrain driveTrain,  Intake intake) {
        _driveTrain = driveTrain;
        _xController = new ProfiledPIDController(2.5, 0.0, 0.0,
                new Constraints(Constants.DriveTrain.SLOW_KINEMATIC_LIMITS.maxDriveVelocity,
                        Constants.DriveTrain.SLOW_KINEMATIC_LIMITS.maxDriveAcceleration));
        _yController = new ProfiledPIDController(2.5, 0.0, 0.0,
                new Constraints(Constants.DriveTrain.SLOW_KINEMATIC_LIMITS.maxDriveVelocity,
                        Constants.DriveTrain.SLOW_KINEMATIC_LIMITS.maxDriveAcceleration));
        _yawController = new ProfiledPIDController(4.0, 0.0, 0.0,
                new Constraints(Constants.DriveTrain.MAX_ANG_VEL, Constants.DriveTrain.MAX_ANG_VEL * 4.0));
        _intake = intake;
        addRequirements(_driveTrain, _intake);
    }

    @Override
    public void initialize() {
        super.initialize();
        isFinished = false;
        _state = IndexState.START;
        _xController.setGoal(0.0);
        _yController.setGoal(0.0);
        _yawController.setGoal(0.0);
        _timeOut = Optional.empty();
        _closestNote = Optional.empty();
    }

    @Override
    public void execute() {
        if (_intake.isNoteIndexed()) {
            _intake.setSpeed(0);
            isFinished = true;
        }
        switch (_state) {
            case START:
                _intake.setSpeed(Constants.Intake.INTAKE_SPEED);
                if (_intake.isMiddleDetected()) {
                    _intake.setSpeed(Constants.Intake.INDEX_SPEED);
                    _state = IndexState.NOTE_DETECTED;
                }
                break;

            case NOTE_DETECTED:
                if (_intake.isTopDetected() && !_intake.isBottomDetected()) {
                    if (_intake.isTopDetected() && _intake.isMiddleDetected()) {
                        _intake.setSpeed(0);
                        _state = IndexState.FINISH;
                    }
                    _intake.setSpeed(Constants.Intake.REVERSE_INDEX_SPEED);
                } else if (!_intake.isTopDetected() && _intake.isBottomDetected()) {
                    _intake.setSpeed(Constants.Intake.INDEX_SPEED);
                } else if (_intake.isTopDetected() && _intake.isBottomDetected()) {
                    _intake.setSpeed(0);
                    _state = IndexState.FINISH;
                } else {
                    _intake.setSpeed(Constants.Intake.INDEX_SPEED);
                }
                break;
            case FINISH:
                _intake.setSpeed(0);
                break;
        }


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
        return isFinished  || _intake.isNoteDetected();  // yolo
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        _intake.setSpeed(0);
    }

    public enum IndexState {
        START(0),
        NOTE_DETECTED(1),
        FINISH(2);

        private final int _value;

        IndexState(int value) {
            _value = value;
        }

        public int getValue() {
            return _value;
        }
    }
}