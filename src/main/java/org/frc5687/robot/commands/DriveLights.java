package org.frc5687.robot.commands;

import static org.frc5687.robot.Constants.DriveTrain.HEADING_TOLERANCE;

import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotState;
import org.frc5687.robot.commands.Intake.IntakeCommand;
import org.frc5687.robot.subsystems.DriveTrain;
import org.frc5687.robot.subsystems.Intake;
import org.frc5687.robot.subsystems.Lights;
import org.frc5687.robot.subsystems.Shooter;
import org.frc5687.robot.subsystems.Lights.AnimationType;
import org.frc5687.robot.util.VisionProcessor;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;



public class DriveLights extends OutliersCommand {
    private Lights _lights;
    private DriveTrain _driveTrain;
    private Intake _intake;
    private VisionProcessor _visionProcessor;
    private RobotState _robotState = RobotState.getInstance();
    private Shooter _shooter;
    
    public DriveLights(Lights lights, DriveTrain driveTrain, Intake intake, VisionProcessor visionProcessor, Shooter shooter) {
        _lights = lights;
        _driveTrain = driveTrain;
        _intake = intake;
        _visionProcessor = visionProcessor;
        _shooter = shooter;
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
        int[] allianceColor = Constants.CANdle.PURPLER; // this should never make the lights purple, like ever, but we need it for the empty check

        if (_shooter.getSpinUpAutomatically()) {
            _lights.setBrightness(Constants.CANdle.SPEAKER_BRIGHTNESS);
        } else {
            _lights.setBrightness(Constants.CANdle.AMP_BRIGHTNESS);
        }

        if (DriverStation.getAlliance().isPresent()) {
            allianceColor = DriverStation.getAlliance().get() == Alliance.Red ? Constants.CANdle.RED : Constants.CANdle.BLUE;
        }

        hasObjects = _visionProcessor.getDetectedObjects().getNotes().length > 0 ? true : false;
        //Not connected to driver station
        if (!DriverStation.isDSAttached()) {
            _lights.switchAnimation(AnimationType.RAINBOW);
        //Disabled, static alliance color
        } else if (DriverStation.isDisabled()) {
            _lights.setColor(allianceColor);
            _lights.switchAnimation(AnimationType.STATIC);
        // Is AutoShooting (shooter debug flag)
        } else if (_lights.getDebugLightsEnabled()) {
            Pair<Double, Double> distanceAndAngle = _robotState.getDistanceAndAngleToSpeaker();

            Rotation2d angle = new Rotation2d(distanceAndAngle.getSecond());
            Rotation2d currentHeading = _driveTrain.getHeading();
            // error("Desired angle: "+angle.getDegrees()+"\n Current angle: "+_driveTrain.getHeading().getDegrees());
            boolean isInAngle = Math.abs(currentHeading.minus(angle).getRadians()) < HEADING_TOLERANCE;
            boolean isAtRPM = _shooter.isAtTargetRPM();
            // show distinct colors for each state
            if (isAtRPM && isInAngle) {
                // both are true
                _lights.setColor(Constants.CANdle.WHITE);
                _lights.switchAnimation(AnimationType.STROBE);
            } else if (isAtRPM) {
                // only rpm is true
                _lights.setColor(Constants.CANdle.PURPLER);
                _lights.switchAnimation(AnimationType.STROBE);
            } else if (isInAngle) {
                // only angle is true
                _lights.setColor(Constants.CANdle.RED);
                _lights.switchAnimation(AnimationType.STROBE);
            } else {
                // neither are true
                _lights.setColor(Constants.CANdle.YELLOW);
                _lights.switchAnimation(AnimationType.STROBE);
            }
        // Is in optimal shooting range
        } else if (_robotState.isWithinOptimalRange()) {
            if (_intake.isTopDetected() || _intake.isBottomDetected() || _intake.isMiddleDetected()) {
                _lights.setColor(Constants.CANdle.GREEN);
            } else {
                _lights.setColor(allianceColor);
            }
            _lights.switchAnimation(AnimationType.STROBE);
        // Is in amp mode
        } else if (_intake.isTopDetected() || _intake.isBottomDetected() || _intake.isMiddleDetected()) {
            _lights.setColor(Constants.CANdle.GREEN);
            _lights.switchAnimation(AnimationType.STATIC);
        //Targeting Note 
        } else if (hasObjects && _driveTrain.getCurrentCommand() != null && _driveTrain.getCurrentCommand().hasRequirement(_intake)) { // I hate this but it works :P
            _lights.setColor(Constants.CANdle.ORANGE);
            _lights.switchAnimation(AnimationType.STROBE);
        //Intaking
        } else if (_intake.getCurrentCommand() instanceof IntakeCommand) {
            _lights.setColor(Constants.CANdle.ORANGE);
            _lights.switchAnimation(AnimationType.STATIC);
        //No other condition is true, use static alliance color (was singlefade)
        } else {
            _lights.setColor(allianceColor);
            _lights.switchAnimation(AnimationType.STATIC);
        }
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
    
}
