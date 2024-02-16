/* Team 5687 (C)2021 */
/* Team 5687 (C)2021-2022 */
package org.frc5687.robot;

import org.frc5687.robot.commands.Drive;
import org.frc5687.robot.commands.DriveLights;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.commands.Climber.AutoClimb;
import org.frc5687.robot.commands.Deflector.IdleDeflector;
import org.frc5687.robot.commands.Intake.IdleIntake;
import org.frc5687.robot.commands.Shooter.IdleShooter;
import org.frc5687.robot.subsystems.*;
import org.frc5687.robot.subsystems.DriveTrain.DriveTrain;
import org.frc5687.robot.util.*;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class RobotContainer extends OutliersContainer {
    private OI _oi;
    private AutoChooser _autoChooser;
    private VisionProcessor _visionProcessor;
    private Pigeon2 _imu;
    private Robot _robot;
    private DriveTrain _driveTrain;
    private Shooter _shooter;
    private Intake _intake;
    private Deflector _deflector;
    private Climber _climber;
    private Lights _lights;

    private Field2d _field;

    private PhotonProcessor _photonProcessor;

    private RobotState _robotState = RobotState.getInstance();

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
        _visionProcessor = new VisionProcessor();
        // subscribe to a vision topic for the correct data
        _visionProcessor.createSubscriber("Objects", "tcp://10.56.87.20:5556");
        _visionProcessor.start();

        _field = new Field2d();

        try {
            _photonProcessor = new PhotonProcessor(AprilTagFields.k2024Crescendo.loadAprilTagLayoutField());
        } catch (Exception e) {
            e.getMessage();
        }

        // configure pigeon
        _imu = new Pigeon2(RobotMap.CAN.PIGEON.PIGEON, "CANivore");
        var pigeonConfig = new Pigeon2Configuration();
        _imu.getConfigurator().apply(pigeonConfig);

        _driveTrain = new DriveTrain(this, _imu);

        // Grab instance such that we can initalize with drivetrain and processor
        _robotState.initializeRobotState(_driveTrain, _photonProcessor);

        _shooter = new Shooter(this);
        _intake = new Intake(this);
        _deflector = new Deflector(this);
        _climber = new Climber(this);
        _lights = new Lights(this);

        setDefaultCommand(_driveTrain, new Drive(_driveTrain, _oi));
        setDefaultCommand(_shooter, new IdleShooter(_shooter));
        setDefaultCommand(_intake, new IdleIntake(_intake));
        setDefaultCommand(_climber, new AutoClimb(_climber, _oi));
        setDefaultCommand(_deflector, new IdleDeflector(_deflector));
        setDefaultCommand(_lights, new DriveLights(_lights, _driveTrain, _intake, _visionProcessor, _robotState));
        
        _oi.initializeButtons(_driveTrain, _shooter, _intake, _deflector, _climber, _visionProcessor, _robotState);
    }

    public void periodic() {
        _robotState.periodic();
        _field.setRobotPose(_robotState.getEstimatedPose());
        SmartDashboard.putData(_field);
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
