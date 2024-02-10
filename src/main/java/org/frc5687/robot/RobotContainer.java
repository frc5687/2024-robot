/* Team 5687 (C)2021 */
/* Team 5687 (C)2021-2022 */
package org.frc5687.robot;

import org.frc5687.robot.commands.Drive;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.commands.Intake.IdleIntake;
import org.frc5687.robot.commands.Shooter.IdleShooter;
import org.frc5687.robot.subsystems.*;
import org.frc5687.robot.util.*;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class RobotContainer extends OutliersContainer {
    private OI _oi;
    private AutoChooser _autoChooser;
    // private VisionProcessor _visionProcessor;
    private Pigeon2 _imu;
    private Robot _robot;
    private DriveTrain _driveTrain;
    private Shooter _shooter;
    private Intake _intake;
    private Deflector _deflector;
    // private Elevator _elevator;
    // private PhotonProcessor _photonProcessor;

    public RobotContainer(Robot robot, IdentityMode identityMode) {
        super(identityMode);
        _robot = robot;
    }

    public void init() {
        Thread.currentThread().setPriority(Thread.MAX_PRIORITY);
        Thread.currentThread().setName("Robot Thread");
        _oi = new OI();
        _autoChooser = new AutoChooser();
        // create the vision processor
        // _visionProcessor = new VisionProcessor();
        // subscribe to a vision topic for the correct data
        // _visionProcessor.createSubscriber("vision", "tcp://10.56.87.20:5557");

        // try {
        // _photonProcessor =
        // // new
        // //
        // PhotonProcessor(AprilTagFieldLayout.loadFromResource("2023-swerret.json"));
        // new PhotonProcessor(FieldConstants.aprilTags);
        // } catch (Exception e) {
        // e.getMessage();
        // }

        // configure pigeon
        TestMotor test1 = new TestMotor(this);
        test1.setSpeed(1.0);
        startPeriodic();

    }

    public void periodic() {
    }

    public void disabledPeriodic() {
        _autoChooser.updateChooser();

    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void teleopInit() {
    }

    @Override
    public void autonomousInit() {
        _autoChooser.updateChooser();
    }

    private void setDefaultCommand(OutliersSubsystem subSystem, OutliersCommand command) {
        if (subSystem == null || command == null) {
            return;
        }
        CommandScheduler s = CommandScheduler.getInstance();
        s.setDefaultCommand(subSystem, command);
    }

    public Command getAutoCommand() {
        return new WaitCommand(15);
    }
}
