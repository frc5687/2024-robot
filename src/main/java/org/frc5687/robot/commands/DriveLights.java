package org.frc5687.robot.commands;

import java.util.Optional;

import org.frc5687.robot.Constants;
import org.frc5687.robot.OI;
import org.frc5687.robot.RobotState;
import org.frc5687.robot.subsystems.Dunker;
import org.frc5687.robot.subsystems.DriveTrain;
import org.frc5687.robot.subsystems.Intake;
import org.frc5687.robot.subsystems.Intake.IndexState;
import org.frc5687.robot.subsystems.Lights;
import org.frc5687.robot.subsystems.Shooter;
import org.frc5687.robot.subsystems.Dunker.DunkerState;
import org.frc5687.robot.subsystems.Lights.AnimationType;
import org.frc5687.robot.subsystems.Lights.LightState;
import org.frc5687.robot.util.VisionProcessor;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class DriveLights extends OutliersCommand {
    private Lights _lights;
    private DriveTrain _driveTrain;
    private Dunker _dunker;
    private Intake _intake;
    private VisionProcessor _visionProcessor;
    private RobotState _robotState = RobotState.getInstance();
    private OI _oi;
    private Shooter _shooter;

    public DriveLights(Lights lights, DriveTrain driveTrain, Intake intake, VisionProcessor visionProcessor,
            Shooter shooter, OI oi, Dunker dunker) {
        _lights = lights;
        _driveTrain = driveTrain;
        _intake = intake;
        _dunker = dunker;
        _visionProcessor = visionProcessor;
        _shooter = shooter;
        _oi = oi;
        addRequirements(lights);
    }

    /*
     * Completed Chain Climb (Just for the style points)
     * In range of speaker (try on regression branch)
     * Has Ring
     * Targeted on Ring
     * Driving normally (alliance color)
     * 
     * Other:
     * Just Shot (Not important)
     * Is In Auto (Might not work)
     */

    @Override
    public void execute() {

        boolean hasObjects = false;
        int[] allianceColor = Constants.CANdle.PURPLER;
        _lights.setBrightness(Constants.CANdle.SPEAKER_BRIGHTNESS);
        if (DriverStation.getAlliance().isPresent()) {
            allianceColor = DriverStation.getAlliance().get() == Alliance.Red ? Constants.CANdle.RED
                    : Constants.CANdle.BLUE;
        }

        hasObjects = _visionProcessor.getDetectedObjects().getNotes().length > 0 ? true : false;
        if (!DriverStation.isDSAttached()) {
            _lights.setLightState(LightState.DISCONNECTED);
        } else if (DriverStation.isDisabled()) {
            _lights.setLightState(LightState.IDLE);
        } else if (_dunker.getDunkerState() == DunkerState.READY_TO_DUNK) {
            _lights.setLightState(LightState.AMP_HANDOFF_COMPLETE);
        } else if (_shooter.getAmpMode() && _intake.isNoteDetected()) {
            _lights.setLightState(LightState.AMP_MODE);
        } else if (_robotState.isWithinOptimalRange() && _intake.isNoteDetected()) {
            _lights.setLightState(LightState.IN_RANGE_SPEAKER);
        }
        else if (_intake.isMiddleDetected()) {
            _lights.setLightState(LightState.HAS_NOTE);
        } else if (hasObjects && _driveTrain.getCurrentCommand() != null
                && _driveTrain.getCurrentCommand().hasRequirement(_intake)) { // I hate this but it works :P
            _lights.setLightState(LightState.FOUND_NOTE);
        } else if (_oi.isIntakeButtonPressed()) {
            _lights.setLightState(LightState.SEEKING_NOTE);
        } else if (_oi.isPassing()) {
            _lights.setLightState(LightState.PASSING);
        } else if (0 == 1) {
            _lights.setLightState(LightState.CLIMBING);
        } else {
            _lights.setLightState(LightState.IDLE);
        }

        switch (_lights.getLightState()) {

            case IDLE:
                _lights.setColor(allianceColor);
                _lights.switchAnimation(AnimationType.STATIC);
                break;

            case SEEKING_NOTE:
                _lights.setColor(Constants.CANdle.ORANGE);
                _lights.switchAnimation(AnimationType.STATIC);
                break;

            case FOUND_NOTE:
                _lights.setColor(Constants.CANdle.ORANGE);
                _lights.switchAnimation(AnimationType.STROBE);
                break;

            case HAS_NOTE:
                _lights.setColor(Constants.CANdle.GREEN);
                _lights.switchAnimation(AnimationType.STATIC);
                break;

            case IN_RANGE_SPEAKER:
                _lights.setColor(Constants.CANdle.GREEN);
                _lights.switchAnimation(AnimationType.STROBE);
                break;

            case AMP_MODE:
                _lights.setColor(Constants.CANdle.PURPLE);
                _lights.switchAnimation(AnimationType.STATIC);
                break;

            case AMP_HANDOFF_COMPLETE:
                _lights.setColor(Constants.CANdle.PURPLE);
                _lights.switchAnimation(AnimationType.STROBE);
                break;

            case PASSING:
                _lights.setColor(Constants.CANdle.GOLD);
                _lights.switchAnimation(AnimationType.STATIC);
                break;

            case CLIMBING:
                _lights.switchAnimation(AnimationType.RGB_FADE);
                break;

            case SHOOTING:
                _lights.setColor(Constants.CANdle.WHITE);
                _lights.switchAnimation(AnimationType.STROBE);
                break;

        }

        // this should never make the lights purple, like ever, but we need it for the
        // empty check

    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

}
