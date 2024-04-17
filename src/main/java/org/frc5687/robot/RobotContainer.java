/* Team 5687 (C)2021-2022 */
package org.frc5687.robot;

import static org.frc5687.robot.Constants.DriveTrain.HIGH_KINEMATIC_LIMITS;
import static org.frc5687.robot.Constants.DriveTrain.LOW_KINEMATIC_LIMITS;

import java.io.IOException;
import java.nio.file.FileSystem;
import java.util.ArrayList;
import java.util.Optional;

import javax.swing.text.html.Option;

import org.frc5687.robot.RobotMap.PDP;
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
import org.frc5687.robot.commands.DriveTrain.DriveToNoteStopBlinded;
import org.frc5687.robot.commands.DriveTrain.DriveToNoteStopNoIndex;
import org.frc5687.robot.commands.DriveTrain.DriveToNoteStopNoIntake;
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
import org.frc5687.robot.commands.Shooter.ManualShoot;
import org.frc5687.robot.commands.Shooter.RevShooter;
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
import org.littletonrobotics.junction.Logger;
// import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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

    private boolean _isRotationOverrideEnabled;

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

        // try {
        //     if (!DriverStation.isFMSAttached()) {
        //         _photonProcessor = new PhotonProcessor(
        //                 new AprilTagFieldLayout("home/lvuser/deploy/layouts/2024-colliseum.json"));
        //         System.out.println("loaded home apriltag layout");
        //     }
        // } catch (IOException e) {
        //     System.out.println(e.getMessage());
        // }

        if (_photonProcessor == null) {
            _photonProcessor = new PhotonProcessor(AprilTagFields.k2024Crescendo.loadAprilTagLayoutField());
            info("loaded comp apriltag layout");
        }

        _isRotationOverrideEnabled = false;

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
        setDefaultCommand(_lights,
                new DriveLights(_lights, _driveTrain, _intake, _visionProcessor, _shooter, _oi, _dunker));

        PathPlannerPath sourcePath = PathPlannerPath.fromPathFile("pathToShootSource");
        PathConstraints sourceConstraints = new PathConstraints(3.0, 4.0, Units.degreesToRadians(540),
                Units.degreesToRadians(720));
        _pathfindSourceSideCommand = AutoBuilder.pathfindThenFollowPath(sourcePath, sourceConstraints, 0.0);

        PathPlannerPath ampPath = PathPlannerPath.fromPathFile("pathToShootAmp");
        PathConstraints ampConstraints = new PathConstraints(3.0, 4.0, Units.degreesToRadians(540),
                Units.degreesToRadians(720));
        _pathfindAmpSideCommand = AutoBuilder.pathfindThenFollowPath(ampPath, ampConstraints, 0.0);

        registerNamedCommands();
        _autoChooser = AutoBuilder.buildAutoChooser("test");
        var firstPath = PathPlannerPath.fromPathFile("Source Side Start to Source Side Shoot");
        var startPose = firstPath.getPreviewStartingHolonomicPose();
        _autoChooser.addOption("Source Side Dynamic Four Piece", new SequentialCommandGroup(
                Commands.runOnce(() -> {
                    _robotState.setEstimatedPose(startPose);
                    _isRotationOverrideEnabled = true;
                }),
                AutoBuilder.followPath(firstPath),
                new WaitCommand(0.01),
                new AutoShoot(_shooter, _intake, _driveTrain, _lights),
                AutoBuilder.followPath(PathPlannerPath.fromPathFile("Xavier's Child shoot-1")),
                new DriveToNoteStop(_driveTrain, _intake),
                // we are at note 1
                new ConditionalCommand(
                        // we have note 1
                        new SequentialCommandGroup(
                                AutoBuilder.followPath(PathPlannerPath.fromPathFile("4p pp.1 part 2")),
                                new WaitCommand(0.01),
                                new AutoShoot(_shooter, _intake, _driveTrain, _lights),
                                AutoBuilder.followPath(PathPlannerPath.fromPathFile("4p pp.2 part 1")),
                                new DriveToNoteStop(_driveTrain, _intake),
                                // we are at note 3
                                new ConditionalCommand(
                                        // we have note 3
                                        new SequentialCommandGroup(
                                                AutoBuilder.followPath(PathPlannerPath.fromPathFile("4p pp.2 part 2")),
                                                new WaitCommand(0.01),
                                                new AutoShoot(_shooter, _intake, _driveTrain, _lights),
                                                AutoBuilder.followPath(PathPlannerPath.fromPathFile("4p pp.3 part 1")),
                                                new DriveToNoteStop(_driveTrain, _intake),
                                                // we are at note 2
                                                AutoBuilder.followPath(PathPlannerPath.fromPathFile("4p pp.3 part 2")),
                                                new WaitCommand(0.01),
                                                new AutoShoot(_shooter, _intake, _driveTrain, _lights)),
                                        // we do not have note 3
                                        new SequentialCommandGroup(
                                                AutoBuilder
                                                        .followPath(PathPlannerPath.fromPathFile("Xavier's Child 3-2")),
                                                AutoBuilder.followPath(PathPlannerPath.fromPathFile("4p pp.3 part 2")),
                                                new AutoShoot(_shooter, _intake, _driveTrain, _lights)),
                                        _intake::isNoteDetected)),
                        // we do not have note 1
                        new SequentialCommandGroup(
                                AutoBuilder.followPath(PathPlannerPath.fromPathFile("Xavier's Child 1-2")),
                                // we are at note 2
                                new ConditionalCommand(
                                        // we have note 2
                                        new SequentialCommandGroup(
                                                AutoBuilder.followPath(PathPlannerPath.fromPathFile("4p pp.3 part 2")),
                                                new WaitCommand(0.01),
                                                new AutoShoot(_shooter, _intake, _driveTrain, _lights),
                                                AutoBuilder.followPath(
                                                        PathPlannerPath.fromPathFile("Xavier's Child amp-3")),
                                                new DriveToNoteStop(_driveTrain, _intake),
                                                AutoBuilder.followPath(PathPlannerPath.fromPathFile("4p pp.2 part 2")),
                                                new WaitCommand(0.01),
                                                new AutoShoot(_shooter, _intake, _driveTrain, _lights)
                                        // end
                                        ),
                                        // we do not have note 2
                                        new SequentialCommandGroup(
                                                AutoBuilder
                                                        .followPath(PathPlannerPath.fromPathFile("Xavier's Child 2-3")),
                                                AutoBuilder.followPath(PathPlannerPath.fromPathFile("4p pp.2 part 2")),
                                                new WaitCommand(0.01),
                                                new AutoShoot(_shooter, _intake, _driveTrain, _lights)
                                        // end
                                        ),
                                        _intake::isNoteDetected)),
                        _intake::isNoteDetected),
                Commands.runOnce(() -> {
                    _isRotationOverrideEnabled = false;
                })));

        _autoChooser.addOption("Jack Dynamic", new SequentialCommandGroup(
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Jack's Child start-1 DX")),
            new ParallelDeadlineGroup(
                new DriveToNoteStopNoIntake(_driveTrain), 
                new AutoPassthrough(_shooter, _intake, 999999)
            ),
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Jack's Child 1-2 D")),
            new DriveToNoteStop(_driveTrain, _intake),
            new ConditionalCommand(
                new SequentialCommandGroup(
                    AutoBuilder.followPath(PathPlannerPath.fromPathFile("Jack's Child 2-shoot")),
                    new AutoShoot(_shooter, _intake, _driveTrain, _lights),
                    AutoBuilder.followPath(PathPlannerPath.fromPathFile("Jack's Child shoot-bloop2 DX"))
                ),
                new WaitCommand(0),
                _intake::isNoteDetected
            ),
            new DriveToNoteStop(_driveTrain, _intake),
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Jack's Child bloop2-shoot")),
            new AutoShoot(_shooter, _intake, _driveTrain, _lights),
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Jack's Child shoot-bloop1 DX")),
            new DriveToNoteStop(_driveTrain, _intake),
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Jack's Child bloop1-shoot DX")),
            new AutoShoot(_shooter, _intake, _driveTrain, _lights)
        ));

        SmartDashboard.putData(_field);
        SmartDashboard.putData("Auto Chooser", _autoChooser);

        _oi.initializeButtons(_driveTrain, _shooter, _dunker, _intake, _climber, _lights, _visionProcessor);

        // PPHolonomicDriveController.setRotationTargetOverride(this::getRotationTargetOverride);
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
        // _robotState.useTeleopStandardDeviations();
        // enforce to make sure kinematic limits are set back to normal.
        _driveTrain.setKinematicLimits(_driveTrain.isLowGear() ? LOW_KINEMATIC_LIMITS : HIGH_KINEMATIC_LIMITS);
    }

    @Override
    public void autonomousInit() {
        // _robotState.useAutoStandardDeviations();
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
        if (_isRotationOverrideEnabled) {
            var visionAngle = _robotState.getAngleToSpeakerFromVision();
            if (visionAngle.isPresent()) {
                return Optional.of(_driveTrain.getHeading().minus(visionAngle.get()));
            }
            return Optional.empty();
        } else {
            return Optional.empty();
        }
    }

    public void registerNamedCommands() {
        // NamedCommands.registerCommand("DynamicNote", new DynamicNotePathCommand());
        NamedCommands.registerCommand("DynamicNote", new DriveToNoteStop(_driveTrain, _intake));
        NamedCommands.registerCommand("DynamicNoteBlinded", new DriveToNoteStopBlinded(_driveTrain, _intake));
        NamedCommands.registerCommand("DynamicNoteNoIndex", new DriveToNoteStopNoIndex(_driveTrain, _intake));
        NamedCommands.registerCommand("DynamicNoteNoIntake", new DriveToNoteStopNoIntake(_driveTrain));
        NamedCommands.registerCommand("EnableRotationOverride", Commands.runOnce(() -> {
            _isRotationOverrideEnabled = true;
        }));
        NamedCommands.registerCommand("DisableRotationOverride", Commands.runOnce(() -> {
            _isRotationOverrideEnabled = false;
        }));
        NamedCommands.registerCommand("ReturnToShoot", new ReturnToShoot());
        NamedCommands.registerCommand("Shoot", new AutoShoot(_shooter, _intake, _driveTrain, _lights));
        NamedCommands.registerCommand("Intake", new AutoIndexNote(_intake)); // was AutoIntake, but IndexNote currently
                                                                             // has the behavior we want
        NamedCommands.registerCommand("Passthrough", new AutoPassthrough(_shooter, _intake, 250));
        NamedCommands.registerCommand("PassthroughNoTimeout", new AutoPassthrough(_shooter, _intake, 99999999));
        NamedCommands.registerCommand("ReturnToShootOpposite", new ReturnToShootOpposite());
        NamedCommands.registerCommand("PassthroughHarder", new AutoPassthroughHarder(_shooter, _intake));
        NamedCommands.registerCommand("Rev", new RevShooter(_shooter));
        NamedCommands.registerCommand("RevStop", new RevShooter(_shooter, 0.0));
        NamedCommands.registerCommand("RevRPM", new RevShooter(_shooter, 2200.0));
        NamedCommands.registerCommand("RevRPMBloopHarder", new RevShooter(_shooter, 1000.0));
        NamedCommands.registerCommand("RevRPMBloop", new RevShooter(_shooter, 460.0));
        NamedCommands.registerCommand("ShootRPM", new DefinedRPMShoot(_shooter, _intake, 1800.0));
        NamedCommands.registerCommand("ShootWhenRPMMatch",
                new ShootWhenRPMMatch(_shooter, _intake, 2150.0, _driveTrain));
        NamedCommands.registerCommand("PathfindToPathAmp", _pathfindAmpSideCommand);
        NamedCommands.registerCommand("PathfindToPathSource", _pathfindSourceSideCommand);
        NamedCommands.registerCommand("EnableVision", new EnableVisionUpdates());
        NamedCommands.registerCommand("DisableVision", new DisableVisionUpdates());

        // auto command groups
        NamedCommands.registerCommand("NoteEightCommand",
                new ConditionalCommand(new NoteEightPickupYes(_shooter, _intake, _driveTrain),
                        new NoteEightPickupNo(_shooter, _intake, _driveTrain), _intake::isNoteDetected));
        NamedCommands.registerCommand("NoteSevenCommand",
                new ConditionalCommand(new NoteSevenPickupYes(_shooter, _intake, _driveTrain),
                        new NoteSevenPickupNo(_shooter, _intake, _driveTrain), _intake::isNoteDetected));
        NamedCommands.registerCommand("NoteSixCommand",
                new ConditionalCommand(new NoteSixPickupYes(_shooter, _intake, _driveTrain),
                        new NoteSixPickupYes(_shooter, _intake, _driveTrain), _intake::isNoteDetected));
        NamedCommands.registerCommand("UnderStageBloopCommand",
                new ConditionalCommand(new UnderStageBloopPickup(_shooter, _intake, _driveTrain), new WaitCommand(0),
                        () -> _robotState.isNoteIntaked(7)));
        NamedCommands.registerCommand("NearSourceBloopCommand",
                new ConditionalCommand(new NearSourceBloopPickup(_shooter, _intake, _driveTrain), new WaitCommand(0),
                        () -> _robotState.isNoteIntaked(8)));

        // NamedCommands.registerCommand("NoteEightCommandChain", new
        // NoteEightCommandChain(_shooter,_driveTrain,_intake));

    }
}