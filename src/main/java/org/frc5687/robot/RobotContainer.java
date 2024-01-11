/* Team 5687 (C)2021 */
/* Team 5687 (C)2021-2022 */
package org.frc5687.robot;

import org.frc5687.robot.commands.Drive;
import org.frc5687.robot.commands.IdleIntake;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.commands.Shooter.IdleShooter;
import org.frc5687.robot.subsystems.*;
import org.frc5687.robot.util.*;
import org.frc5687.robot.util.AutoChooser.AutoType;
import org.frc5687.robot.util.AutoChooser.Node;

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
    // private Pigeon2 _imu;
    private Robot _robot;
    // private Lights _lights;
    // private LightsExample _lights;
    // private DriveTrain _driveTrain;
    private Shooter _shooter;
    private Intake _intake;
    // private EndEffector _endEffector;
    // private CubeShooter _cubeShooter;
    // private Arm _arm;
    // private Elevator _elevator;
    // private PhotonProcessor _photonProcessor;
    // private Trajectories _trajectories;

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
        // _trajectories = new Trajectories(new PathConstraints(3.0, 2.0));

        // try {
        //     _photonProcessor =
        //             // new
        //             // PhotonProcessor(AprilTagFieldLayout.loadFromResource("2023-swerret.json"));
        //             new PhotonProcessor(FieldConstants.aprilTags);
        // } catch (Exception e) {
        //     e.getMessage();
        // }
        // configure pigeon
        // _imu = new Pigeon2(RobotMap.CAN.PIGEON.PIGEON, "CANivore");
        // var pigeonConfig = new Pigeon2Configuration();
        // _imu.getConfigurator().apply(pigeonConfig);

        // _driveTrain = new DriveTrain(this, /*_visionProcessor, */_photonProcessor, _imu);
        _shooter = new Shooter(this);
        _intake = new Intake(this);

        // setDefaultCommand(_driveTrain, new Drive(_driveTrain, /*_endEffector,*/ _oi));
        setDefaultCommand(_shooter, new IdleShooter(_shooter));
        setDefaultCommand(_intake, new IdleIntake(_intake));
        _oi.initializeButtons(/*_driveTrain,*/ _shooter, _intake);
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
