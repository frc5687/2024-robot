package org.frc5687.robot.commands;

import org.frc5687.robot.Constants;
import org.frc5687.robot.OI;
import org.frc5687.robot.RobotState;
import org.frc5687.robot.subsystems.Dunker;
import org.frc5687.robot.subsystems.DriveTrain;
import org.frc5687.robot.subsystems.Intake;
import org.frc5687.robot.subsystems.Lights;
import org.frc5687.robot.subsystems.Shooter;
import org.frc5687.robot.subsystems.Dunker.DunkerState;
import org.frc5687.robot.subsystems.Lights.AnimationType;
import org.frc5687.robot.util.VisionProcessor;

import edu.wpi.first.wpilibj.DriverStation;

public class DriveLights extends OutliersCommand {
    private Lights _lights;
    private DriveTrain _driveTrain;
    private Dunker _dunker;
    private Intake _intake;
    private VisionProcessor _visionProcessor;
    private RobotState _robotState = RobotState.getInstance();
    private OI _oi;
    private Shooter _shooter;

    public DriveLights(
        Lights lights, 
        DriveTrain driveTrain, 
        Intake intake, 
        VisionProcessor visionProcessor,
        Shooter shooter, 
        OI oi, 
        Dunker dunker
    ) {
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

        boolean hasObjects = _visionProcessor.getDetectedObjects().getNotes().length > 0 ? true : false;
        boolean isRedAlliance = _driveTrain.isRedAlliance();
        int[] allianceColor = isRedAlliance ? Constants.CANdle.RED : Constants.CANdle.BLUE;
        _lights.setBrightness(Constants.CANdle.BRIGHTNESS);

        if (!DriverStation.isDSAttached()) {
            // no driver station is connected
            _lights.switchAnimation(AnimationType.RAINBOW);
        } else if (DriverStation.isDisabled()) {
            // driver station conencted, but disabled
            _lights.setColor(allianceColor);
            if (Math.abs(_driveTrain.getHeading().minus(isRedAlliance ? Constants.DriveTrain.RED_SHOOT_ROTATION : Constants.DriveTrain.BLUE_SHOOT_ROTATION).getRadians()) < Constants.DriveTrain.HEADING_TOLERANCE) {
                // is aligned with first shot angle
                _lights.switchAnimation(AnimationType.STATIC);
            } else {
                // isn't aligned with first shot angle
                _lights.switchAnimation(AnimationType.STROBE);
            }
        } else if (_dunker.getDunkerState() == DunkerState.READY_TO_DUNK){
            // dunker in dunking position
            _lights.setColor(Constants.CANdle.PURPLE);
            _lights.switchAnimation(AnimationType.STROBE);
        } else if (_shooter.getAmpMode() && _intake.isNoteDetected()){
            // in amp mode with a note
            _lights.setColor(Constants.CANdle.PURPLE);
            _lights.switchAnimation(AnimationType.STATIC);
        } else if (_robotState.isWithinOptimalRange() && _intake.isNoteDetected()) {
            // in shooting range with note
            _lights.setColor(Constants.CANdle.GREEN);
            _lights.switchAnimation(AnimationType.STROBE);
        } else if (_intake.isNoteDetected()) {
            // has a note
            _lights.setColor(Constants.CANdle.GREEN);
            _lights.switchAnimation(AnimationType.STATIC);
        } else if (hasObjects && _driveTrain.getCurrentCommand() != null && _driveTrain.getCurrentCommand().hasRequirement(_intake)) { // I hate this but it works :P
            // intaking with vision and sees a note
            _lights.setColor(Constants.CANdle.ORANGE);
            _lights.switchAnimation(AnimationType.STROBE);
        } else if (_oi.isIntakeButtonPressed()) {
            // manual intaking
            _lights.setColor(Constants.CANdle.ORANGE);
            _lights.switchAnimation(AnimationType.STATIC);
        } else if (_oi.isPassing()) {
            // passing
            _lights.setColor(Constants.CANdle.GOLD);
            _lights.switchAnimation(AnimationType.STATIC);
        } else {
            // enabled, but not running any of the above commands
            _lights.setColor(allianceColor);
            _lights.switchAnimation(AnimationType.STATIC);
        }
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

}
