/* Team 5687 (C)2020-2022 */
package org.frc5687.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.frc5687.lib.logging.ILoggingSource;
import org.frc5687.lib.logging.MetricTracker;
import org.frc5687.lib.logging.RioLogger;
import org.frc5687.robot.util.*;

/**
 * Base class to provide metrics and logging infrustructure.
 *
 * <p>See ILoggingSource for logging details.
 *
 * <p>See MetricTracker for metrics details.
 *
 * <p>Example of metrics collection. In a child class's constructor:
 *
 * <p>public SampleSystem() { logMetrics("foo", "bar", "baz"); }
 *
 * <p>Later on in the child class, eg in setSpeed():
 *
 * <p>... metric("foo", 123); metric("bar", "elvis"); metric("baz", 42.42); metric("pants", 99); <~
 * This metric won't get written to USB storage because it wasn't registered. ...
 *
 * <p>Note that metric logging is expensive, so it is turned OFF by default. To turn it on, either
 * call enableMetrics() from code (eg at the start of an auto command) or set
 * Metrics/[SubSystemName] to true in the SmartDashboard
 */
public abstract class OutliersSubsystem extends SubsystemBase implements ILoggingSource {
    private MetricTracker _metricTracker;

    public OutliersSubsystem(OutliersContainer container) {
        container.registerSubsystem(this);
    }

    @Override
    public void error(String message) {
        RioLogger.error(this, message);
    }

    @Override
    public void warn(String message) {
        RioLogger.warn(this, message);
    }

    @Override
    public void info(String message) {
        RioLogger.info(this, message);
    }

    @Override
    public void debug(String message) {
        RioLogger.debug(this, message);
    }

    public void metric(String name, String value) {
        SmartDashboard.putString(getClass().getSimpleName() + "/" + name, value);
        if (_metricTracker != null) {
            _metricTracker.put(name, value);
        }
    }

    public void metric(String name, double value) {
        SmartDashboard.putNumber(getClass().getSimpleName() + "/" + name, value);
        if (_metricTracker != null) {
            _metricTracker.put(name, value);
        }
    }

    public void metric(String name, boolean value) {
        SmartDashboard.putBoolean(getClass().getSimpleName() + "/" + name, value);
        if (_metricTracker != null) {
            _metricTracker.put(name, value);
        }
    }

    protected void logMetrics(String... metrics) {
        _metricTracker = MetricTracker.createMetricTracker(getClass().getSimpleName(), metrics);
    }

    public abstract void updateDashboard();

    public void enableMetrics() {
        if (_metricTracker != null) {
            _metricTracker.enable();
        }
    }

    public void disableMetrics() {
        if (_metricTracker != null) {
            _metricTracker.disable();
        }
    }
}
