/* Team 5687 (C)2021-2022 */
package org.frc5687.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.frc5687.lib.control.SwerveHeadingController;
import org.frc5687.lib.control.SwerveHeadingController.HeadingState;
import org.frc5687.lib.math.Vector2d;
import org.frc5687.robot.Constants;
import org.frc5687.robot.OI;
import org.frc5687.robot.subsystems.DriveTrain;
import org.frc5687.robot.subsystems.DriveTrain.Mode;
import org.frc5687.robot.util.Helpers;

public class Drive extends OutliersCommand {

    private final DriveTrain _driveTrain;
    // private final EndEffector _endEffector;
    // private final HeadingController _headingController;
    private final PIDController _yCordinateElementController;
    private final OI _oi;
    private boolean _lockHeading;
    private boolean _isOverride = false;
    private boolean _toNormal = false;
    private int segmentationArray[] = new int[360 / 5];

    public Drive(DriveTrain driveTrain, OI oi) {
        _lockHeading = false;
        _driveTrain = driveTrain;
        // _endEffector = endEffector;
        _oi = oi;
        _yCordinateElementController = new PIDController(2.5, 0.0, 0.3);
        // _headingController = new HeadingController(
        // new TrapezoidProfile.Constraints(
        // Constants.DriveTrain.PROFILE_CONSTRAINT_VEL,
        // Constants.DriveTrain.PROFILE_CONSTRAINT_ACCEL
        // )
        // );

        for (int i = 0; i < segmentationArray.length; i++) {
            double angle = 360 / segmentationArray.length;
            segmentationArray[i] = (int) angle * i;
        }
        addRequirements(_driveTrain);
    }

    @Override
    public void initialize() {
        // _driveTrain.startModules();
        // _headingController.setGoal(_driveTrain.getHeading().getRadians());
        _driveTrain.setControlState(DriveTrain.ControlState.MANUAL);
    }

    @Override
    public void execute() {
        if (_oi.zeroIMU()) {
            _driveTrain.zeroGyroscope();
            _driveTrain.setHeadingControllerState(SwerveHeadingController.HeadingState.OFF);
            _lockHeading = false;
        }
        if (_oi.shiftUp()) {
            _driveTrain.shiftUpModules();
        } else if (_oi.shiftDown()) {
            _driveTrain.shiftDownModules();
        } else if (!_isOverride) {
            _driveTrain.autoShifter();
        }
        // driveX and driveY are swapped due to coordinate system that WPILib uses.
        Vector2d vec = Helpers.axisToSegmentedUnitCircleRadians(
                _oi.getDriveY(), _oi.getDriveX(), segmentationArray);
        double vx;
        double vy;
        double rot = _oi.getRotationX();
        rot = Math.signum(rot) * rot * rot;
        // driveX and driveY are swapped due to coordinate system that WPILib uses
        if (rot == 0 && _driveTrain.getHeadingControllerState() != HeadingState.SNAP) {
            if (!_lockHeading) {
                _driveTrain.temporaryDisabledHeadingController();
            }
            _lockHeading = true;
        } else if (_driveTrain.getHeadingControllerState() != HeadingState.SNAP) {
            _driveTrain.disableHeadingController();
            _lockHeading = false;
        }

        double controllerPower = _driveTrain.getRotationCorrection();
        // metric("Element Angle", elementAngle);
        metric("Rot+Controller", (rot + controllerPower));
        
        if (!_toNormal) {
            _driveTrain.setKinematicLimits(Constants.DriveTrain.LOW_KINEMATIC_LIMITS);
            _toNormal = true;
        }
        _driveTrain.setMode(Mode.NORMAL);
        // _driveTrain.setKinematicLimits(Constants.DriveTrain.KINEMATIC_LIMITS);
        // _driveTrain.setKinematicLimits(Constants.DriveTrain.HIGH_KINEMATIC_LIMITS);
        // _driveTrain.setShiftLockout(false);
        vx = vec.x() * (_driveTrain.isLowGear() ? Constants.DriveTrain.MAX_LOW_GEAR_MPS
                : Constants.DriveTrain.MAX_HIGH_GEAR_MPS);
        vy = vec.y() * (_driveTrain.isLowGear() ? Constants.DriveTrain.MAX_LOW_GEAR_MPS
                : Constants.DriveTrain.MAX_HIGH_GEAR_MPS);
        rot = rot * Constants.DriveTrain.MAX_ANG_VEL;
        _driveTrain.setVelocity(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        vx, vy, rot + controllerPower, _driveTrain.getHeading()));
        SmartDashboard.putNumber("/vx", vx);
        SmartDashboard.putNumber("/vy", vy);

    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}
