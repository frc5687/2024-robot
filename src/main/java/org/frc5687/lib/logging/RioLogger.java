/* Team 5687 (C)2021-2022 */
package org.frc5687.lib.logging;

import edu.wpi.first.wpilibj.DriverStation;
import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.TimeZone;

// Imported from RobotCasserole2017
public class RioLogger {
    private static RioLogger _instance;

    public static RioLogger getInstance() {
        if (_instance == null) {
            _instance = new RioLogger();
        }
        return _instance;
    }

    public static void warn(String source, String message) {
        log(LogLevel.warn, source, message);
    }

    public static void error(String source, String message) {
        log(LogLevel.error, source, message);
    }

    public static void debug(String source, String message) {
        log(LogLevel.debug, source, message);
    }

    public static void info(String source, String message) {
        log(LogLevel.info, source, message);
    }

    public static void warn(Object source, String message) {
        warn(source.getClass().getSimpleName(), message);
    }

    public static void error(Object source, String message) {
        error(source.getClass().getSimpleName(), message);
    }

    public static void debug(Object source, String message) {
        debug(source.getClass().getSimpleName(), message);
    }

    public static void info(Object source, String message) {
        info(source.getClass().getSimpleName(), message);
    }

    public static void log(LogLevel level, String source, String message) {
        getInstance().logint(level, source, message);
    }

    String log_name = null;
    String output_dir = "/U/"; // USB drive is mounted to /U on roboRIO
    BufferedWriter log_file = null;
    boolean log_open = false;

    private LogLevel _fileLogLevel;
    private LogLevel _dsLogLevel;

    public int init(LogLevel fileLogLevel, LogLevel dsLogLevel) {
        _fileLogLevel = fileLogLevel;
        _dsLogLevel = dsLogLevel;

        if (_fileLogLevel == LogLevel.none) {
            return 0;
        }

        if (log_open) {
            System.out.println("Warning - log is already open!");
            return 0;
        }

        log_open = false;
        try {
            // Determine a unique file name
            log_name = output_dir + "log_" + getDateTimeString() + ".txt";
            // Open File
            FileWriter fstream = new FileWriter(log_name, true);
            log_file =
                    new BufferedWriter(fstream); // 8K by default. Probably big enough but size is 2nd arg if
            // not.
            // End of line
            log_file.write("\n\r");
            log_open = true;
        }
        // Catch ALL the errors!!!
        catch (IOException e) {

            System.out.println("Error initializing log file: " + e.getMessage());
            return -1;
        }

        log_open = true;

        return 0;
    }

    private String getDateTimeString() {
        DateFormat df = new SimpleDateFormat("yyyy-MM-dd_HHmmss");
        df.setTimeZone(TimeZone.getTimeZone("US/Eastern"));
        return df.format(new Date());
    }

    protected void logint(LogLevel level, String source, String message) {
        if (level.getValue() >= _dsLogLevel.getValue()) {
            DriverStation.reportError(level.toString() + "\t" + source + "\t" + message, false);
        }
        if (_fileLogLevel == LogLevel.none) {
            return;
        }

        if (level.getValue() >= _fileLogLevel.getValue()) {
            writeData(getDateTimeString(), level.toString(), source, message);
        }
    }

    public int writeData(String... data_elements) {
        StringBuilder line_to_write = new StringBuilder();

        if (log_open == false) {
            System.out.println("Error - Log is not yet opened, cannot write!");
            return -1;
        }

        try {

            // Write user-defined data
            for (String data_val : data_elements) {
                line_to_write.append(data_val).append(" ");
            }

            // End of line
            line_to_write.append("\r\n");

            // write constructed string out to file

            log_file.write(line_to_write.toString());
        }
        // Catch ALL the errors!!!
        catch (IOException e) {
            System.out.println("Error writing to log file: " + e.getMessage());
            return -1;
        }
        return 0;
    }

    /**
     * Clears the buffer in memory and forces things to file. Generally a good idea to use this as
     * infrequently as possible (because it increases logging overhead), but definitely use it before
     * the roboRIO might crash without a proper call to the close() method (during brownout, maybe?)
     *
     * @return Returns 0 on flush success or -1 on failure.
     */
    public int forceSync() {
        if (log_open == false) {
            System.out.println("Error - Log is not yet opened, cannot sync!");
            return -1;
        }
        try {
            log_file.flush();
        }
        // Catch ALL the errors!!!
        catch (IOException e) {
            System.out.println("Error flushing IO stream file: " + e.getMessage());
            return -1;
        }

        return 0;
    }

    /**
     * Closes the log file and ensures everything is written to disk. init() must be called again in
     * order to write to a new file.
     *
     * @return -1 on failure to close, 0 on success
     */
    public int close() {

        if (log_open == false) {
            System.out.println("Warning - Log is not yet opened, nothing to close.");
            return 0;
        }

        try {
            log_file.close();
            log_open = false;
        }
        // Catch ALL the errors!!!
        catch (IOException e) {
            System.out.println("Error Closing Log File: " + e.getMessage());
            return -1;
        }
        return 0;
    }

    public enum LogLevel {
        none(0),
        debug(1),
        info(2),
        warn(3),
        error(4);

        private int _value;

        LogLevel(int value) {
            this._value = value;
        }

        public int getValue() {
            return _value;
        }
    }
}
