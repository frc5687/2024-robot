/* (C)2020-2021 */
package org.frc5687.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import org.frc5687.lib.logging.ILoggingSource;
import org.frc5687.lib.logging.MetricTracker;
import org.frc5687.lib.logging.RioLogger;

public abstract class OutliersCommand extends Command implements ILoggingSource {
    private MetricTracker _metricTracker;

    public OutliersCommand() {}

    public OutliersCommand(double timeout) {
        super.withTimeout(timeout);
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
        _metricTracker.pause();
    }

    @Override
    public void initialize() {
        super.initialize();
        if (_metricTracker != null) {
            _metricTracker.resume();
        }
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        if (_metricTracker != null) {
            _metricTracker.pause();
        }
    }

    @Override
    public void execute() {
        if (_metricTracker != null && _metricTracker.isPaused()) {
            _metricTracker.resume();
        }
    }

    protected void enableMetrics() {
        if (_metricTracker != null) {
            _metricTracker.enable();
        }
    }

    protected void disableMetrics() {
        if (_metricTracker != null) {
            _metricTracker.disable();
        }
    }

    protected void innerExecute() {}
}
