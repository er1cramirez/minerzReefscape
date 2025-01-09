package frc.robot.telemetry;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public interface SubsystemTelemetry extends Sendable {
    /**
     * Initialize telemetry data and properties
     * @param builder The SendableBuilder to configure
     */
    void initTelemetry(SendableBuilder builder);

    /**
     * Update any telemetry that needs periodic updates
     */
    void updateTelemetry();

    /**
     * Default implementation of Sendable's initSendable that calls our initTelemetry
     */
    @Override
    default void initSendable(SendableBuilder builder) {
        initTelemetry(builder);
    }
}