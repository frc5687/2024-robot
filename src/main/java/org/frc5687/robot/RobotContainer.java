/* Team 5687 (C)2021-2022 */
package org.frc5687.robot;

import static org.frc5687.robot.Constants.DriveTrain.HIGH_KINEMATIC_LIMITS;
import static org.frc5687.robot.Constants.DriveTrain.LOW_KINEMATIC_LIMITS;

import java.util.ArrayList;
import java.util.Optional;

import org.frc5687.robot.commands.DisableVisionUpdates;
import org.frc5687.robot.commands.DriveLights;
import org.frc5687.robot.commands.EnableVisionUpdates;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.commands.AutoCommands.NearSourceBloopPickup;
import org.frc5687.robot.commands.AutoCommands.NoteEightPickupNo;
import org.frc5687.robot.commands.AutoCommands.NoteEightPickupYes;
import org.frc5687.robot.commands.AutoCommands.NoteSevenPickupNo;
import org.frc5687.robot.commands.AutoCommands.NoteSevenPickupYes;
import org.frc5687.robot.commands.AutoCommands.NoteSixPickupYes;
import org.frc5687.robot.commands.AutoCommands.UnderStageBloopPickup;
import org.frc5687.robot.commands.Climber.AutoClimb;
import org.frc5687.robot.commands.DriveTrain.Drive;
import org.frc5687.robot.commands.DriveTrain.DriveToNoteStop;
import org.frc5687.robot.commands.DriveTrain.ReturnToShoot;
import org.frc5687.robot.commands.DriveTrain.ReturnToShootOpposite;
import org.frc5687.robot.commands.Dunker.IdleDunker;
import org.frc5687.robot.commands.Intake.AutoIndexNote;
import org.frc5687.robot.commands.Intake.IndexNote;
import org.frc5687.robot.commands.Shooter.AutoPassthrough;
import org.frc5687.robot.commands.Shooter.AutoPassthroughHarder;
import org.frc5687.robot.commands.Shooter.AutoShoot;
import org.frc5687.robot.commands.Shooter.ShootWhenRPMMatch;
import org.frc5687.robot.commands.Shooter.DefinedRPMShoot;
import org.frc5687.robot.commands.Shooter.IdleShooter;
import org.frc5687.robot.commands.Shooter.RevShooter;
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
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

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
    Command _pathfindSourceSideCommand;
    Command _pathfindAmpSideCommand;


    public RobotContainer(Robot robot, IdentityMode identityMode) {
        super(identityMode);
        _robot = robot;
    }

    public void init() {
        Thread.currentThread().setPriority(Thread.MAX_PRIORITY);
        Thread.currentThread().setName("Robot Thread");

        Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
        Logger.start();

        _oi = new OI();
        // create the vision processor
        _visionProcessor = new VisionProcessor();

        _field = new Field2d();

        _photonProcessor = new PhotonProcessor(AprilTagFields.k2024Crescendo.loadAprilTagLayoutField());

        // configure pigeon
        _imu = new Pigeon2(RobotMap.CAN.PIGEON.PIGEON, "CANivore");
        var pigeonConfig = new Pigeon2Configuration();
        _imu.getConfigurator().apply(pigeonConfig);

        _driveTrain = new DriveTrain(this, _imu);

        _shooter = new Shooter(this);
        _intake = new Intake(this);
        _dunker = new Dunker(this);
        _climber = new Climber(this);
        _lights = new Lights(this);

        _robotState.initializeRobotState(_driveTrain, _photonProcessor, _visionProcessor);
        _robotState.start();

        setDefaultCommand(_driveTrain, new Drive(_driveTrain, _oi, _intake, _shooter));
        setDefaultCommand(_shooter, new IdleShooter(_shooter, _intake));
        setDefaultCommand(_dunker, new IdleDunker(_dunker));
        setDefaultCommand(_intake, new IndexNote(_intake, _oi));
        setDefaultCommand(_climber, new AutoClimb(_climber, _dunker, _driveTrain, _oi));
        setDefaultCommand(_lights, new DriveLights(_lights, _driveTrain, _intake, _visionProcessor, _shooter, _oi,_dunker));
        
        PathPlannerPath sourcePath = PathPlannerPath.fromPathFile("pathToShootSource");
        PathConstraints sourceConstraints = new PathConstraints(3.0, 4.0,Units.degreesToRadians(540), Units.degreesToRadians(720));
            _pathfindSourceSideCommand = AutoBuilder.pathfindThenFollowPath(sourcePath,sourceConstraints,0.0);

        PathPlannerPath ampPath = PathPlannerPath.fromPathFile("pathToShootAmp");
        PathConstraints ampConstraints = new PathConstraints(3.0, 4.0,Units.degreesToRadians(540), Units.degreesToRadians(720));
            _pathfindAmpSideCommand = AutoBuilder.pathfindThenFollowPath(ampPath,ampConstraints,0.0);

        registerNamedCommands();
        _autoChooser = AutoBuilder.buildAutoChooser("");

        SmartDashboard.putData(_field);
        SmartDashboard.putData("Auto Chooser", _autoChooser);

        _oi.initializeButtons(_driveTrain, _shooter, _dunker, _intake, _climber, _lights, _visionProcessor);

        PPHolonomicDriveController.setRotationTargetOverride(this::getRotationTargetOverride);
    }

    public void periodic() {
    }

    public void disabledPeriodic() {
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void updateDashboard() {
        super.updateDashboard();
        _field.setRobotPose(_robotState.getEstimatedPose());

        // var notes = _robotState.getAllNotesRlativeField();
        var notes = _robotState.getAllNotesRelativeField();

        if (notes.isPresent()) {
            var fieldNotes = _field.getObject("notes");
            var notePoses = new ArrayList<Pose2d>();

            for (var note : notes.get()) {
                notePoses.add(note);
            }

            fieldNotes.setPoses(notePoses);
        } else {
            var fieldNotes = _field.getObject("notes");
            fieldNotes.setPoses(new ArrayList<Pose2d>()); // Clear all notes by setting an empty list of poses
        }
        Pair<EstimatedRobotPose, String>[] cameraPoses = _robotState.getLatestCameraPoses();

        for (int i = 0; i < cameraPoses.length; i++) {
            Pair<EstimatedRobotPose, String> cameraPose = cameraPoses[i];

            if (cameraPose != null) {
                Logger.recordOutput("RobotState/CameraPose/" + cameraPose.getSecond(),
                        cameraPose.getFirst().estimatedPose.toPose2d());
            }
        }

        Logger.recordOutput("RobotState/EstimatedRobotPose", _robotState.getEstimatedPose());
        // _field.getObject("futurePose").setPose(_robotState.calculateAdjustedRPMAndAngleToTargetPose());
        SmartDashboard.putData(_field);
    }

    @Override
    public void teleopInit() {
        _driveTrain.enableAutoShifter();
        _robotState.useTeleopStandardDeviations();
        // enforce to make sure kinematic limits are set back to normal.
        _driveTrain.setKinematicLimits(_driveTrain.isLowGear() ? LOW_KINEMATIC_LIMITS : HIGH_KINEMATIC_LIMITS);
    }

    @Override
    public void autonomousInit() {
        _robotState.useAutoStandardDeviations();
        _driveTrain.setKinematicLimits(Constants.DriveTrain.AUTO_KINEMATIC_LIMITS);
        // yolo
        _driveTrain.disableAutoShifter();
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
        var visionAngle = _robotState.getAngleToSpeakerFromVision();
        if (visionAngle.isPresent()) {
            return Optional.of(_driveTrain.getHeading().minus(visionAngle.get()));
        }
        return Optional.empty();
        // // Some condition that should decide if we want to override rotation
        // if (_shooter.getAutoShootFlag()) {
        // // Return an optional containing the rotation override (this should be a
        // field
        // // relative rotation)
        // return Optional.of(new
        // Rotation2d(_robotState.getDistanceAndAngleToSpeaker().getSecond()));
        // } else {
        // // return an empty optional when we don't want to override the path's
        // rotation
        // return Optional.empty();
        // }

    }

    public void registerNamedCommands() {
        // NamedCommands.registerCommand("DynamicNote", new DynamicNotePathCommand());
        NamedCommands.registerCommand("DynamicNote", new DriveToNoteStop(_driveTrain, _intake));
        NamedCommands.registerCommand("ReturnToShoot", new ReturnToShoot());
        NamedCommands.registerCommand("Shoot", new AutoShoot(_shooter, _intake, _driveTrain, _lights));
        NamedCommands.registerCommand("Intake", new AutoIndexNote(_intake)); // was AutoIntake, but IndexNote currently
                                                                             // has the behavior we want
        NamedCommands.registerCommand("Passthrough", new AutoPassthrough(_shooter, _intake, 250));
        NamedCommands.registerCommand("ReturnToShootOpposite", new ReturnToShootOpposite());
        NamedCommands.registerCommand("PassthroughHarder", new AutoPassthroughHarder(_shooter, _intake));
        NamedCommands.registerCommand("Rev", new RevShooter(_shooter));
        NamedCommands.registerCommand("RevRPM", new RevShooter(_shooter, 2200.0));
        NamedCommands.registerCommand("RevRPMBloopHarder", new RevShooter(_shooter, 1000.0));
        NamedCommands.registerCommand("RevRPMBloop", new RevShooter(_shooter, 460.0));
        NamedCommands.registerCommand("ShootRPM", new DefinedRPMShoot(_shooter, _intake, 2150.0));
        NamedCommands.registerCommand("ShootWhenRPMMatch",
                new ShootWhenRPMMatch(_shooter, _intake, 2150.0, _driveTrain));
        NamedCommands.registerCommand("PathfindToPathAmp", _pathfindAmpSideCommand);
        NamedCommands.registerCommand("PathfindToPathSource", _pathfindSourceSideCommand);
        NamedCommands.registerCommand("EnableVision", new EnableVisionUpdates());
        NamedCommands.registerCommand("DisableVision", new DisableVisionUpdates());

        //auto command groups
        NamedCommands.registerCommand("NoteEightCommand", new ConditionalCommand(new NoteEightPickupYes(_shooter, _intake, _driveTrain), new NoteEightPickupNo(_shooter, _intake, _driveTrain), _intake::isNoteDetected));
        NamedCommands.registerCommand("NoteSevenCommand", new ConditionalCommand(new NoteSevenPickupYes(_shooter, _intake, _driveTrain),new NoteSevenPickupNo(_shooter, _intake, _driveTrain), _intake::isNoteDetected));
        NamedCommands.registerCommand("NoteSixCommand", new ConditionalCommand(new NoteSixPickupYes(_shooter, _intake, _driveTrain), new NoteSixPickupYes(_shooter, _intake, _driveTrain) , _intake::isNoteDetected));
        NamedCommands.registerCommand("UnderStageBloopCommand", new ConditionalCommand(new UnderStageBloopPickup(_shooter, _intake, _driveTrain), new WaitCommand(0), () -> _robotState.isNoteIntaked(7)));
        NamedCommands.registerCommand("NearSourceBloopCommand", new ConditionalCommand(new NearSourceBloopPickup(_shooter, _intake, _driveTrain), new WaitCommand(0), () -> _robotState.isNoteIntaked(8)));




        //NamedCommands.registerCommand("NoteEightCommandChain", new NoteEightCommandChain(_shooter,_driveTrain,_intake));

    }
}