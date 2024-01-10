/* Team 5687 (C)2021-2022 */
package org.frc5687.lib.logging;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.HashMap;
import java.util.Map;
import java.util.TimeZone;

public class MetricTracker {
    private static final int BUFFER_LENGTH = 500;
    private static Map<String, MetricTracker> _allMetricsTrackers = new HashMap<>();

    private Map<String, Integer> _metrics = new HashMap<String, Integer>();
    private Object[][] _metricBuffer;
    private int _metricCount = 0;

    private boolean _streamOpen = false;
    private BufferedWriter _bufferedWriter;

    private int _in = 0;
    private int _out = 0;

    private boolean _paused = false;
    private boolean _bufferOverflowed = false;
    private long _startTime = System.currentTimeMillis();

    private String _instrumentedClassName;
    /**
     * MetricsTracker factory method - Creates a new Metrics Tracker and registers the associated
     * object with a list of metrics so they can all be flushed to SD by calling the static flushAll
     * method.
     *
     * @param instrumentedClassName name of the class being measured - used to generate the metric
     *     file name.
     * @param metrics list of the metric names to track. Other metrics will be ignored.
     * @return a fresh new MetricTracker.
     */
    public static MetricTracker createMetricTracker(String instrumentedClassName, String... metrics) {
        if (_allMetricsTrackers.containsKey(instrumentedClassName)) {
            return _allMetricsTrackers.get(instrumentedClassName);
        }

        SmartDashboard.putBoolean("MetricTracker/" + instrumentedClassName, false);
        MetricTracker newMetricTracker = new MetricTracker(instrumentedClassName, metrics);
        MetricTracker._allMetricsTrackers.put(instrumentedClassName, newMetricTracker);
        return newMetricTracker;
    }

    /**
     * Shorthand version of the ctor
     *
     * @param instrumentedObject name of the class being measured - used to generate the metric file
     *     name.
     * @param metrics list of the metric names to track. Other metrics will be ignored.
     * @return a fresh new MetricTracker.
     */
    public static MetricTracker createMetricTracker(Object instrumentedObject, String... metrics) {
        return createMetricTracker(instrumentedObject.getClass().getSimpleName(), metrics);
    }

    /**
     * Starts a new row for all instrumented objects. This is called by robotPeriodic once per cycle.
     */
    public static void newMetricRowAll() {
        for (MetricTracker metricTracker : MetricTracker._allMetricsTrackers.values()) {
            metricTracker.newMetricRow();
        }
    }

    /**
     * Called by a notifier to periodically flush all all known metrics trackers the rows of metrics
     * to perm storage.
     */
    public static void flushAll() {
        for (MetricTracker metricTracker : MetricTracker._allMetricsTrackers.values()) {
            metricTracker.flushMetricsTracker();
        }
    }

    /**
     * Private ctor. Call createMetricsTracker.
     *
     * @param instrumentedClassName The name of the instrumented class. This is used to name the
     *     output file.
     * @param metrics list of the metric names to track. Other metrics will be ignored.
     */
    private MetricTracker(String instrumentedClassName, String... metrics) {
        _instrumentedClassName = instrumentedClassName;
        StringBuilder header = new StringBuilder();
        _metricCount = metrics.length;

        int index = 0;
        header.append("timestamp");
        for (String metric : metrics) {
            _metrics.put(metric, index);
            header.append(",");
            header.append(metric);
            index++;
        }

        _metricBuffer = new Object[BUFFER_LENGTH][_metricCount + 1];
        _in = 0;
        _out = 0;

        // Don't use the c'tor. Use createMetricTracker.
        String outputDir = "/U/"; // USB drive is symlinked to /U on roboRIO
        String filename = outputDir + instrumentedClassName + "_" + getDateTimeString() + ".csv";

        try {
            _bufferedWriter = new BufferedWriter(new FileWriter(filename, true));
            _bufferedWriter.append(header.toString());
            _bufferedWriter.newLine();

            _streamOpen = true;

        } catch (IOException e) {
            System.out.println("Error initializing metrics file: " + e.getMessage());
        }
    }

    /**
     * Adds a metric (name,value) pair to this row of metrics
     *
     * @param name
     * @param value
     */
    public void put(String name, Object value) {
        // If this tracker is paused, just return.
        if (_paused) {
            return;
        }

        // If our buffer is full, don't try to write to it
        if (_bufferOverflowed) {
            return;
        }

        // Find the metric index
        int index = _metrics.getOrDefault(name, -1);

        // If not found, exit (ie, this metric isn't flagged for logging)
        if (index < 0) {
            return;
        }

        int row = -1;
        synchronized (this) {
            row = _in;
        }
        // Record the metric (the +1 is because the first column is the timestamp)
        _metricBuffer[row][index + 1] = value;
    }

    /**
     * Pause collection of metrics by this tracker. This does NOT prevent writing already-buffered
     * metrics to the file!
     */
    public void pause() {
        _paused = true;
    }

    /**
     * Resume collection of metrics by this tracker. This does NOT prevent writing already-buffered
     * metrics to the file!
     */
    public void resume() {
        if (SmartDashboard.getBoolean("MetricTracker/" + _instrumentedClassName, false)) {
            _paused = false;
        }
    }

    public void enable() {
        SmartDashboard.putBoolean("MetricTracker/" + _instrumentedClassName, true);
    }

    public void disable() {
        SmartDashboard.putBoolean("MetricTracker/" + _instrumentedClassName, false);
    }

    /**
     * Starts a new row of metrics. You'd call this, e.g., once per tick. Normally this is called from
     * the static newMetricRowAll method, so there's no need to call it directly.
     */
    protected void newMetricRow() {
        if (!_streamOpen || _paused) {
            return;
        }
        // Synchronized for threadsafety
        synchronized (this) {
            if (_out == _in + 1 || (_out == 0 && _in == BUFFER_LENGTH - 1)) {
                // Buffer overflow...we can't log any new metrics until the writer catches up!
                _bufferOverflowed = true;
                return;
            }
            _in++;
            if (_in >= BUFFER_LENGTH) {
                _in = 0;
            }
            _bufferOverflowed = false;
        }
        _metricBuffer[_in][0] = (_startTime - System.currentTimeMillis()) / 1000.0;
        for (int index = 0; index < _metricCount; index++) {
            _metricBuffer[_in][index + 1] = null;
        }
    }

    /**
     * Helper function to determine in a threadsafe way whether there are rows to write. We know we've
     * written all the rows when _out reaches _in.
     *
     * @return
     */
    private boolean hasUnwrittenRows() {
        synchronized (this) {
            return _out != _in;
        }
    }

    /** Flushes the buffer of stats for an instance of a metrics tracker to perm storage. */
    protected void flushMetricsTracker() {
        if (_bufferedWriter == null) {
            return;
        }
        // Keep writing rows as long at there are rows left to write...
        while (hasUnwrittenRows()) {
            writeMetricRow(_out);
            synchronized (this) {
                _out++;
                if (_out >= BUFFER_LENGTH) {
                    _out = 0;
                }
            }
        }

        try {
            _bufferedWriter.flush();
        } catch (IOException e) {
            System.out.println("Error closing metrics file: " + e.getMessage());
        }
    }

    /** Formats a row of metrics as a comma-delimited quoted string. */
    private void writeMetricRow(int row) {
        if (_bufferedWriter == null) {
            return;
        }
        try {
            for (int i = 0; i <= _metricCount; i++) {
                if (i > 0) {
                    _bufferedWriter.write(",");
                }
                if (_metricBuffer[row][i] != null) {
                    _bufferedWriter.write(_metricBuffer[row][i].toString());
                }
            }
            _bufferedWriter.newLine();
        } catch (IOException e) {
            System.out.println("Error writing metrics file: " + e.getMessage());
        }
    }

    /**
     * Creates a timestamp to include in the log file name.
     *
     * @return Formatted timestamp
     */
    private String getDateTimeString() {
        DateFormat df = new SimpleDateFormat("yyyyMMdd_HHmmss");
        df.setTimeZone(TimeZone.getTimeZone("US/Eastern"));
        return df.format(new Date());
    }

    public boolean isPaused() {
        return _paused;
    }
}
