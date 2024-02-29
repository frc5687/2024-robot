package org.frc5687.robot.commands;

import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotState;
import org.frc5687.robot.commands.Intake.IntakeCommand;
import org.frc5687.robot.subsystems.DriveTrain;
import org.frc5687.robot.subsystems.Intake;
import org.frc5687.robot.subsystems.Lights;
import org.frc5687.robot.subsystems.Shooter;
import org.frc5687.robot.subsystems.Lights.AnimationType;
import org.frc5687.robot.util.VisionProcessor;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;



public class DriveLights extends OutliersCommand {
    private Lights _lights;
    private DriveTrain _driveTrain;
    private Intake _intake;
    private VisionProcessor _visionProcessor;
    private RobotState _robotState;
    private Shooter _shooter;
    public DriveLights(Lights lights, DriveTrain driveTrain, Intake intake, VisionProcessor visionProcessor, RobotState robotState, Shooter shooter) {
        _lights = lights;
        _driveTrain = driveTrain;
        _intake = intake;
        _visionProcessor = visionProcessor;
        _robotState = robotState;
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
        _lights.setBrightness(1);
        boolean hasObjects = false;
        try {
            hasObjects = _visionProcessor.getDetectedObjects().posesLength() > 0;
        } catch (Exception e) {
            // error("No objects");
        }
        //Not connected to driver station
        if (!DriverStation.isDSAttached()) {
            _lights.switchAnimation(AnimationType.RAINBOW);
        //Disabled, static alliance color
        } else if (DriverStation.isDisabled()) {
            _lights.setColor(DriverStation.getAlliance().get() == Alliance.Red ? Constants.CANdle.RED : Constants.CANdle.BLUE);
            _lights.switchAnimation(AnimationType.STATIC);
        // Is in amp mode
        } else if (!_shooter.getSpinUpAutomatically()) {
            _lights.setColor(Constants.CANdle.PURPLER);
            _lights.switchAnimation(AnimationType.STATIC);
        // Is in optimal shooting range
        } else if (_robotState.isWithinOptimalRange() && (_intake.isTopDetected() || _intake.isBottomDetected())) {
            _lights.setColor(Constants.CANdle.GREEN);
            _lights.switchAnimation(AnimationType.STROBE);
        //Has Note
        } else if (_intake.isTopDetected() || _intake.isBottomDetected()) {
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
        //No other condition is true, use breathing alliance color
        } else {
            _lights.setColor(
                DriverStation.getAlliance().get() == Alliance.Red ? Constants.CANdle.RED : Constants.CANdle.BLUE
            );
            _lights.switchAnimation(AnimationType.SINGLE_FADE);
        }
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
    
}
