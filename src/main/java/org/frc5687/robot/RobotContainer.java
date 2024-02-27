/* Team 5687 (C)2021-2022 */
package org.frc5687.robot;

import java.util.Optional;

import org.frc5687.robot.commands.DriveLights;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.commands.Climber.AutoClimb;
import org.frc5687.robot.commands.DriveTrain.Drive;
import org.frc5687.robot.commands.Dunker.IdleDunker;
import org.frc5687.robot.commands.Intake.AutoIntake;
import org.frc5687.robot.commands.Intake.IdleIntake;
import org.frc5687.robot.commands.Intake.IntakeCommand;
import org.frc5687.robot.commands.Shooter.AutoShoot;
import org.frc5687.robot.commands.Shooter.IdleShooter;
import org.frc5687.robot.commands.Shooter.Shoot;
import org.frc5687.robot.subsystems.Climber;
import org.frc5687.robot.subsystems.DriveTrain;
import org.frc5687.robot.subsystems.Dunker;
import org.frc5687.robot.subsystems.Intake;
import org.frc5687.robot.subsystems.Lights;
import org.frc5687.robot.subsystems.OutliersSubsystem;
import org.frc5687.robot.subsystems.Shooter;
import org.frc5687.robot.util.OutliersContainer;
import org.frc5687.robot.util.PhotonProcessor;
import org.frc5687.robot.util.VisionProcessor;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class RobotContainer extends OutliersContainer {
    private OI _oi;
    private SendableChooser<Command> _autoChooser;
    private VisionProcessor _visionProcessor;
    private Pigeon2 _imu;
    private Robot _robot;
    private DriveTrain _driveTrain;
    private Shooter _shooter;
    private Intake _intake;
    private Dunker _dunker;
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
        // create the vision processor
        _visionProcessor = new VisionProcessor();
        // subscribe to a vision topic for the correct data
        _visionProcessor.createSubscriber("Objects", "tcp://10.56.87.20:5556");
        _visionProcessor.start();

        _field = new Field2d();

        _photonProcessor = new PhotonProcessor(AprilTagFields.k2024Crescendo.loadAprilTagLayoutField());


        // configure pigeon
        _imu = new Pigeon2(RobotMap.CAN.PIGEON.PIGEON, "CANivore");
        var pigeonConfig = new Pigeon2Configuration();
        _imu.getConfigurator().apply(pigeonConfig);

        _driveTrain = new DriveTrain(this, _imu);

        // Grab instance such that we can initalize with drivetrain and processor
        _robotState.initializeRobotState(_driveTrain, _photonProcessor, _visionProcessor);

        _shooter = new Shooter(this);
        _intake = new Intake(this);
        _dunker = new Dunker(this);

        _climber = new Climber(this);
        _lights = new Lights(this);

        setDefaultCommand(_driveTrain, new Drive(_driveTrain, _oi));
        setDefaultCommand(_shooter, new IdleShooter(_shooter, _dunker, _intake));
        setDefaultCommand(_dunker, new IdleDunker(_dunker));
        setDefaultCommand(_intake, new IdleIntake(_intake));
        setDefaultCommand(_climber, new AutoClimb(_climber, _dunker, _driveTrain, _oi));
        setDefaultCommand(_lights, new DriveLights(_lights, _driveTrain, _intake, _visionProcessor, _robotState));

        registerNamedCommands();
        _autoChooser = AutoBuilder.buildAutoChooser("");

        SmartDashboard.putData(_field);
        SmartDashboard.putData("Auto Chooser", _autoChooser);

        _oi.initializeButtons(_driveTrain, _shooter, _dunker, _intake, _climber, _visionProcessor, _robotState);

        PPHolonomicDriveController.setRotationTargetOverride(this::getRotationTargetOverride);
    }

    public void periodic() {
        _robotState.periodic();
        _field.setRobotPose(_robotState.getEstimatedPose());
        // Optional<Pose2d> optionalClosestNote = _robotState.getClosestNote();
        // if (optionalClosestNote.isPresent()) {
        //     Pose2d notePose = optionalClosestNote.get();
        //     _field.getObject("note").setPose(notePose);
        // } else {
        //     // TODO remove note from glass
        // }
        SmartDashboard.putData(_field);
    }

    public void disabledPeriodic() {
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void teleopInit() {
        _robotState.useTeleopStandardDeviations();
    }

    @Override
    public void autonomousInit() {
        _robotState.useAutoStandardDeviations();
    }

    private void setDefaultCommand(OutliersSubsystem subSystem, OutliersCommand command) {
        if (subSystem == null || command == null) {
            return;
        }
        CommandScheduler s = CommandScheduler.getInstance();
        s.setDefaultCommand(subSystem, command);
    }

    public Command getAutoCommand() {
        // // Follow a path
        // // Load the path you want to follow using its name in the GUI
        // PathPlannerPath path = PathPlannerPath.fromPathFile("C3_TO_SHOOT_TO_F3");

        // // Create a path following command using AutoBuilder. This will also trigger
        // event markers.
        // return AutoBuilder.followPath(path);
        // // return _autoChooser.getSelected();

        /*
         * Warning
         * This method will load all autos in the deploy directory. Since the deploy
         * process does not automatically clear the deploy directory, old auto files
         * that have since been deleted from the project could remain on the RIO,
         * therefore being added to the auto chooser.
         * To remove old options, the deploy directory will need to be cleared manually
         * via SSH, WinSCP, reimaging the RIO, etc.
         */

        return _autoChooser.getSelected();
    }

    public Optional<Rotation2d> getRotationTargetOverride() {
        // // Some condition that should decide if we want to override rotation
        // if (_shooter.getAutoShootFlag()) {
        //     // Return an optional containing the rotation override (this should be a field
        //     // relative rotation)
        //     return Optional.of(new Rotation2d(_robotState.getDistanceAndAngleToSpeaker().getSecond()));
        // } else {
        //     // return an empty optional when we don't want to override the path's rotation
            return Optional.empty();
        // }

    }

    public void registerNamedCommands() {
        NamedCommands.registerCommand("Shoot", new AutoShoot(_shooter, _intake, _driveTrain, _robotState));
        NamedCommands.registerCommand("Intake", new AutoIntake(_intake));

    }
}