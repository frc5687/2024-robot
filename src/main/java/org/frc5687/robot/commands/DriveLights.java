package org.frc5687.robot.commands;

import static org.frc5687.robot.Constants.DriveTrain.HEADING_TOLERANCE;

import java.util.Optional;

import org.frc5687.robot.Constants;
import org.frc5687.robot.OI;
import org.frc5687.robot.RobotState;
import org.frc5687.robot.commands.Climber.Climb;
import org.frc5687.robot.commands.DriveTrain.DriveToNote;
import org.frc5687.robot.commands.Shooter.Pass;
import org.frc5687.robot.commands.Shooter.Shoot;
import org.frc5687.robot.subsystems.Dunker;
import org.frc5687.robot.subsystems.Climber;
import org.frc5687.robot.subsystems.DriveTrain;
import org.frc5687.robot.subsystems.Intake;
import org.frc5687.robot.subsystems.Intake.IndexState;
import org.frc5687.robot.subsystems.Lights;
import org.frc5687.robot.subsystems.Shooter;
import org.frc5687.robot.subsystems.Dunker.DunkerState;
import org.frc5687.robot.subsystems.Lights.AnimationType;
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
    private Climber _climber;
    private VisionProcessor _visionProcessor;
    private RobotState _robotState = RobotState.getInstance();
    private OI _oi;
    private Shooter _shooter;
    
    public DriveLights(Lights lights, DriveTrain driveTrain, Intake intake, VisionProcessor visionProcessor, Shooter shooter, OI oi, Dunker dunker, Climber climber) {
        _lights = lights;
        _driveTrain = driveTrain;
        _intake = intake;
        _dunker = dunker;
        _visionProcessor = visionProcessor;
        _shooter = shooter;
        _climber = climber;
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
        allianceColor = DriverStation.getAlliance().get() == Alliance.Red ? Constants.CANdle.RED : Constants.CANdle.BLUE;
    }

        hasObjects = _visionProcessor.getDetectedObjects().getNotes().length > 0;
        if (!DriverStation.isDSAttached()) {
            _lights.switchAnimation(AnimationType.RAINBOW);
        } else if (DriverStation.isDisabled()) {
            _lights.setColor(allianceColor);
            _lights.switchAnimation(AnimationType.STATIC);
        } else if (_dunker.getDunkerState() == DunkerState.READY_TO_DUNK){
            _lights.setColor(Constants.CANdle.PURPLE);
            _lights.switchAnimation(AnimationType.STROBE);
        } else if (!_shooter.getSpinUpAutomatically()){
            _lights.setColor(Constants.CANdle.PURPLE);
            _lights.switchAnimation(AnimationType.STATIC);
        } else if (_robotState.isWithinOptimalRange() && _intake.isNoteDetected()) {
            _lights.setColor(Constants.CANdle.GREEN);
            _lights.switchAnimation(AnimationType.STROBE);
        } else if (_shooter.getCurrentCommand() instanceof Shoot) {
            _lights.setColor(Constants.CANdle.WHITE);
            _lights.switchAnimation(AnimationType.STROBE);
        } else if (_intake.isNoteDetected()) {
            _lights.setColor(Constants.CANdle.GREEN);
            _lights.switchAnimation(AnimationType.STATIC);
        } else if (hasObjects && _intake.getCurrentCommand() instanceof DriveToNote) {
            _lights.setColor(Constants.CANdle.ORANGE);
            _lights.switchAnimation(AnimationType.STROBE);
        } else if (_oi.isIntakeButtonPressed()) {
            _lights.setColor(Constants.CANdle.ORANGE);
            _lights.switchAnimation(AnimationType.STATIC);
        } else if (_shooter.getCurrentCommand() instanceof Pass) {
            _lights.setColor(Constants.CANdle.GOLD);
            _lights.switchAnimation(AnimationType.STATIC);
        } else if (_climber.getCurrentCommand() instanceof Climb) {
            _lights.switchAnimation(AnimationType.RAINBOW);
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
