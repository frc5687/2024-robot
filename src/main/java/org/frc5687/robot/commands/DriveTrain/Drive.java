/* Team 5687 (C)2021-2022 */
package org.frc5687.robot.commands.DriveTrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import static org.frc5687.robot.Constants.DriveTrain.HIGH_KINEMATIC_LIMITS;
import static org.frc5687.robot.Constants.DriveTrain.LOW_KINEMATIC_LIMITS;

import org.frc5687.lib.control.SwerveHeadingController.HeadingState;
import org.frc5687.lib.math.Vector2d;
import org.frc5687.robot.Constants;
import org.frc5687.robot.OI;
import org.frc5687.robot.RobotState;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.DriveTrain;
import org.frc5687.robot.subsystems.Intake;
import org.frc5687.robot.subsystems.Shooter;
import org.frc5687.robot.util.Helpers;

public class Drive extends OutliersCommand {

    private final DriveTrain _driveTrain;
    private final OI _oi;
    // TODO: move some of these to robotState for more elegant shared state. - xavier bradford 03/01/24
    private final Intake _intake;
    private final Shooter _shooter;
    private final RobotState _robotState = RobotState.getInstance();
    private int segmentationArray[] = new int[360 / 5];

    public Drive(DriveTrain driveTrain, OI oi, Intake intake, Shooter shooter) {
        _driveTrain = driveTrain;
        _oi = oi;
        _intake = intake;
        _shooter = shooter;

        for (int i = 0; i < segmentationArray.length; i++) {
            double angle = 360 / segmentationArray.length;
            segmentationArray[i] = (int) angle * i;
        }
        addRequirements(_driveTrain);
    }

    @Override
    public void initialize() {
        _driveTrain.setMaintainHeading(_driveTrain.getHeading());
        _driveTrain.setHeadingControllerState(HeadingState.MAINTAIN);
        _driveTrain.setControlState(DriveTrain.ControlState.MANUAL);
    }

    @Override
    public void execute() {

        if (_oi.zeroIMU()) {
            _driveTrain.zeroGyroscope();
            _driveTrain.setHeadingControllerState(HeadingState.OFF);
            _driveTrain.setLockHeading(false);
        }

        /*if (_oi.shiftUp()) {
            _driveTrain.shiftUpModules();
        } else*/ 

        if (_driveTrain.isShiftingDown()) {
            _driveTrain.shiftDownModules();
        } else {
            _driveTrain.autoShifter();
        }

        Vector2d vec = Helpers.axisToSegmentedUnitCircleRadians(
                _oi.getDriveY(), _oi.getDriveX(), segmentationArray);
        double vx;
        double vy;
        double rot = _oi.getRotationX();

        double max_mps = _driveTrain.isLowGear() ? 
                 Constants.DriveTrain.MAX_LOW_GEAR_MPS
                : Constants.DriveTrain.MAX_HIGH_GEAR_MPS;

        rot = Math.signum(rot) * rot * rot;

        if (
            rot == 0 && 
            (_driveTrain.getHeadingControllerState() != HeadingState.SNAP || 
            _driveTrain.getHeadingControllerState() != HeadingState.TRACKING)) {
            if (!_driveTrain.isHeadingLocked()) {
                _driveTrain.temporaryDisabledHeadingController();
            }
            _driveTrain.setLockHeading(true);
        } else if (
            _driveTrain.getHeadingControllerState() != HeadingState.SNAP || 
            _driveTrain.getHeadingControllerState() != HeadingState.TRACKING) {

            _driveTrain.disableHeadingController();
            _driveTrain.setLockHeading(false);
        }

        double controllerPower = _driveTrain.getRotationCorrection();

        vx = vec.x() * max_mps;
        vy = vec.y() * max_mps;
        rot = rot * Constants.DriveTrain.MAX_ANG_VEL;

        Rotation2d rotation = _driveTrain.isRedAlliance() ? _driveTrain.getHeading().plus(new Rotation2d(Math.PI)) : _driveTrain.getHeading();

        // set kinematics limits if shooting.
        // if (_oi.isShooting()) {
        // } else {
        //     _driveTrain.setKinematicLimits(
        //         _driveTrain.isLowGear() ? 
        //         LOW_KINEMATIC_LIMITS :
        //         HIGH_KINEMATIC_LIMITS
        //     );
        // }

        // if has note and is within shooting range and is in speaker mode
        boolean shouldAutoAim = (_intake.isBottomDetected() || _intake.isTopDetected()) && _robotState.isWithinOptimalRange() && _shooter.getSpinUpAutomatically();
        
        metric("Auto Aiming at Speaker?", shouldAutoAim);

        // note: this uses maintain heading not snap heading
        if (shouldAutoAim) {
            _driveTrain.setMaintainHeading(new Rotation2d(_robotState.getDistanceAndAngleToSpeaker().getSecond()));
        }

        if (_driveTrain.isFieldCentric()) {
            _driveTrain.setVelocity(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    vx, vy, rot + controllerPower,
                    rotation
                )
            );
        } else {
            _driveTrain.setVelocity(
                ChassisSpeeds.fromRobotRelativeSpeeds(
                    vx, vy, rot + controllerPower,
                    rotation
                )
            );
        }
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
