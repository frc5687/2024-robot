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
        _intake = intake;
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
        //Not connected to driver station
        if (!DriverStation.isDSAttached()) {
            _lights.switchAnimation(AnimationType.RAINBOW);
        //Has Note
        } else if (_intake.isTopDetected()) {
            _lights.setColor(Constants.CANdle.GREEN);
            _lights.switchAnimation(AnimationType.STATIC);
        //Targeting Note
        /* } else if (_vision.hasTarget()) { //unimplemented
            _lights.setColor(Constants.CANdle.ORANGE);
            _lights.switchAnimation(AnimationType.STROBE);
        */
        //Intaking
        } else if (_intake.getCurrentCommand() instanceof IntakeCommand) {
            _lights.setColor(Constants.CANdle.ORANGE);
            _lights.switchAnimation(AnimationType.STATIC);
        //No other condition is true, use alliance color
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
