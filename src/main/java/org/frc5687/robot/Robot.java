/* Team 5687 (C)2020-2022 */
package org.frc5687.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;

import org.frc5687.lib.logging.MetricTracker;
import org.frc5687.lib.logging.RioLogger;
import org.frc5687.robot.util.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends OutliersRobot {

    public static OutliersContainer.IdentityMode _identityMode = OutliersContainer.IdentityMode.competition;
    private RioLogger.LogLevel _dsLogLevel = RioLogger.LogLevel.warn;
    private RioLogger.LogLevel _fileLogLevel = RioLogger.LogLevel.warn;

    private int _updateTick = 0;

    private String _name;

    private RobotContainer _robotContainer;

    private boolean _fmsConnected;

    private Command _autoCommand;

    private Timer _timer;


    /**
     * This function is setRollerSpeed when the robot is first started up and should
     * be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        loadConfigFromUSB();
        RioLogger.getInstance().init(_fileLogLevel, _dsLogLevel);
        LiveWindow.disableAllTelemetry();
        DriverStation.silenceJoystickConnectionWarning(true);

        metric("Identity", _identityMode.toString());
        info("Robot " + _name + " running in " + _identityMode.toString() + " mode");

        _robotContainer = new RobotContainer(this, _identityMode);
        _timer = new Timer();
        _robotContainer.init();

        // Periodically flushes metrics (might be good to configure enable/disable via
        // USB config
        // file)
        new Notifier(MetricTracker::flushAll).startPeriodic(Constants.METRIC_FLUSH_PERIOD);
    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for
     * items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and
     * test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        ourPeriodic();
    }

    /**
     * This autonomous (along with the chooser code above) shows how to select
     * between different
     * autonomous modes using the dashboard. The sendable chooser code works with
     * the Java
     * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the
     * chooser code and
     * uncomment the getString line to get the auto name from the text box below the
     * Gyro
     *
     * <p>
     * You can add additional auto modes by adding additional comparisons to the
     * switch structure
     * below with additional strings. If using the SendableChooser make sure to add
     * them to the
     * chooser code above as well.
     */
    @Override
    public void autonomousInit() {
        _fmsConnected = DriverStation.isFMSAttached();
        _robotContainer.autonomousInit();
        _autoCommand = _robotContainer.getAutoCommand();
        if (_autoCommand != null) {
        _autoCommand.schedule();
        }
    }

    public void teleopInit() {
        _fmsConnected = DriverStation.isFMSAttached();
        _robotContainer.teleopInit();

        // _limelight.disableLEDs();
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
    }

    private void ourPeriodic() {

        // Example of starting a new row of metrics for all instrumented objects.
        // MetricTracker.newMetricRowAll();
        MetricTracker.newMetricRowAll();
        _robotContainer.periodic();
        CommandScheduler.getInstance().run();
        update();
        updateDashboard();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
        // _limelight.disableLEDs();
        RioLogger.getInstance().forceSync();
        RioLogger.getInstance().close();
        _robotContainer.disabledInit();
        // MetricTracker.flushAll();
    }

    @Override
    public void disabledPeriodic() {
        super.disabledPeriodic();
        _robotContainer.disabledPeriodic();
    }

    public void updateDashboard() {
        _updateTick++;
        if (_updateTick >= Constants.TICKS_PER_UPDATE) {
            _updateTick = 0;
            _robotContainer.updateDashboard();
        }
    }

    private void loadConfigFromUSB() {
        String output_dir = "/U/"; // USB drive is mounted to /U on roboRIO
        try {
            String usbDir = "/U/"; // USB drive is mounted to /U on roboRIO
            String configFileName = usbDir + "frc5687.cfg";
            File configFile = new File(configFileName);
            FileReader reader = new FileReader(configFile);
            BufferedReader bufferedReader = new BufferedReader(reader);

            String line;
            while ((line = bufferedReader.readLine()) != null) {
                processConfigLine(line);
            }

            bufferedReader.close();
            reader.close();
        } catch (Exception e) {
            _identityMode = OutliersContainer.IdentityMode.competition;
        }
    }

    private void processConfigLine(String line) {
        try {
            if (line.startsWith("#")) {
                return;
            }
            String[] a = line.split("=");
            if (a.length == 2) {
                String key = a[0].trim().toLowerCase();
                String value = a[1].trim();
                switch (key) {
                    case "name":
                        _name = value;
                        break;
                    case "mode":
                        _identityMode = OutliersContainer.IdentityMode.valueOf(value.toLowerCase());
                        break;
                    case "fileloglevel":
                        _fileLogLevel = RioLogger.LogLevel.valueOf(value.toLowerCase());
                        break;
                    case "dsloglevel":
                        _dsLogLevel = RioLogger.LogLevel.valueOf(value.toLowerCase());
                        break;
                }
            }
        } catch (Exception e) {

        }
    }

    private void update() {
    }
}
