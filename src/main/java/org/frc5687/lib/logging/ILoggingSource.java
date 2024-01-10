/* Team 5687 (C)2021-2022 */
package org.frc5687.lib.logging;

public interface ILoggingSource {
    void error(String message);

    void warn(String message);

    void info(String message);

    void debug(String message);
}
