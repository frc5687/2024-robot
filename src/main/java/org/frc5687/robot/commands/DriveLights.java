package org.frc5687.robot.commands;

import org.frc5687.robot.Constants;
import org.frc5687.robot.commands.Intake.IntakeCommand;
import org.frc5687.robot.subsystems.DriveTrain;
import org.frc5687.robot.subsystems.Intake;
import org.frc5687.robot.subsystems.Lights;
import org.frc5687.robot.subsystems.Lights.AnimationType;
import org.frc5687.robot.util.OutliersContainer;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;



public class DriveLights extends OutliersCommand {
    private Lights _lights;
    private DriveTrain _driveTrain;
    private Intake _intake;
    public DriveLights(Lights lights, DriveTrain driveTrain, Intake intake) {
        _lights = lights;
        _driveTrain = driveTrain;
        addRequirements(lights);
    }
    
    /*
     * Has Ring
     * Targeted on Ring
     * Targeted Speaker/amp
     * Driving normally
     * Completed Chain Climb (Just for the style points)
     * 
     * Just Shot (Not important)
     * Is In Auto (Might not work)
     */

    @Override
    public void execute() {
        if (DriverStation.isDisabled()) {
            var alliance = DriverStation.getAlliance();
            if (!DriverStation.isDSAttached()) {
                _lights.switchAnimation(AnimationType.RAINBOW);
                return;
            }
            if (alliance.get() == Alliance.Blue) {
                _lights.setColor(Constants.CANdle.BLUE);
            } else {
                _lights.setColor(Constants.CANdle.RED);
            }
            _lights.switchAnimation(AnimationType.SINGLE_FADE);
        } else {
            //Has Ring
            if (_intake.isTopDetected()) {
                _lights.setColor(Constants.CANdle.ORANGE);
                _lights.setBrightness(1);
                _lights.switchAnimation(AnimationType.STATIC);
            } else if (_intake.getCurrentCommand() instanceof IntakeCommand) {
                _lights.setColor(Constants.CANdle.ORANGE);
                _lights.switchAnimation(AnimationType.STROBE);
            //Targeting Ring
            /* } else if (_lockHeading = true) {
                _lights.setColor(Constants.CANdle.LESS_GREEN);
                _lights.setBrightness(0.8);
                _lights.switchAnimation(AnimationType.LARSON);

            //Targeting Goals
            /* } else if (LockedOnToSpeaker/Amp = true) {
                _lights.setColor(Constants.CANdle.MILO_BLUE);
                _lights.setBrightness(0.8);
                _lights.switchAnimation(AnimationType.LARSON);
            */

            //Driving Contant Speed
            }  else if (_driveTrain.getSpeed() > 1) {
                _lights.setColor(Constants.CANdle.GREEN);
                _lights.setBrightness(0.6);
                _lights.switchAnimation(AnimationType.STROBE);

            //Completed Climb
            /*}  else if (CompletedChainClimb = true ) {
                _lights.setColor(Constants.CANdle.LEAF00);
                _lights.setBrightness(0.5);
                _lights.switchAnimation(AnimationType.TWINKLE);


            */ } else {
                 _lights.setColor(Constants.CANdle.RUFOUS);
                _lights.setBrightness(1);
                _lights.switchAnimation(AnimationType.STATIC);
            }
        }
    }

    

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
    
}
