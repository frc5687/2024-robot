/* Team 5687 (C)2021-2022 */
package org.frc5687.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.frc5687.lib.control.SwerveHeadingController;
import org.frc5687.lib.control.SwerveHeadingController.HeadingState;
import org.frc5687.lib.math.Vector2d;
import org.frc5687.robot.Constants;
import org.frc5687.robot.OI;
import org.frc5687.robot.subsystems.DriveTrain.DriveTrain;
import org.frc5687.robot.util.Helpers;

public class Drive extends OutliersCommand {

    private final DriveTrain _driveTrain;
    private final OI _oi;
    private boolean _isOverride = false;
    private int segmentationArray[] = new int[360 / 5];

    public Drive(DriveTrain driveTrain, OI oi) {
        _driveTrain = driveTrain;
        // _endEffector = endEffector;
        _oi = oi;

        for (int i = 0; i < segmentationArray.length; i++) {
            double angle = 360 / segmentationArray.length;
            segmentationArray[i] = (int) angle * i;
        }
        addRequirements(_driveTrain);
    }

    @Override
    public void initialize() {
        _driveTrain.setControlState(DriveTrain.ControlState.MANUAL);
        _driveTrain.setEnableHeadingController(true);
    }

    @Override
    public void execute() {
        if (_oi.zeroIMU()) {
            _driveTrain.zeroGyroscope();
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

        vx = vec.x() * (_driveTrain.isLowGear() ? Constants.DriveTrain.MAX_LOW_GEAR_MPS
                : Constants.DriveTrain.MAX_HIGH_GEAR_MPS);
        vy = vec.y() * (_driveTrain.isLowGear() ? Constants.DriveTrain.MAX_LOW_GEAR_MPS
                : Constants.DriveTrain.MAX_HIGH_GEAR_MPS);
        rot = rot * Constants.DriveTrain.MAX_ANG_VEL;
        _driveTrain.setVelocity(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                vx, vy, rot,
                _driveTrain.getHeading()
            )
        );
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
