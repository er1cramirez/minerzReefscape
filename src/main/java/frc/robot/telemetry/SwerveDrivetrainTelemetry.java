package frc.robot.telemetry;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveDrivetrain;

public class SwerveDrivetrainTelemetry implements SubsystemTelemetry {
    private final SwerveDrivetrain swerveDrive;

    public SwerveDrivetrainTelemetry(SwerveDrivetrain swerveDrive) {
        this.swerveDrive = swerveDrive;
    }

    /**
     * Converts module states to an array format expected by the YAGSL widget
     * @param states Array of SwerveModuleStates to convert
     * @return array of alternating angles (rad) and velocities (m/s)
     */
    private double[] convertToTelemetryArray(SwerveModuleState[] states) {
        double[] telemetryArray = new double[states.length * 2];
        for (int i = 0; i < states.length; i++) {
            telemetryArray[i * 2] = states[i].angle.getRadians();
            telemetryArray[i * 2 + 1] = states[i].speedMetersPerSecond;
        }
        return telemetryArray;
    }

    @Override
    public void initTelemetry(SendableBuilder builder) {
        builder.setSmartDashboardType("YAGSL Swerve Drive");
        
        builder.addDoubleArrayProperty("measuredStates", 
            () -> convertToTelemetryArray(swerveDrive.getCurrentStates()), null);
        builder.addDoubleArrayProperty("desiredStates",
            () -> convertToTelemetryArray(swerveDrive.getTargetStates()), null);

        builder.addDoubleProperty("robotRotation",
            () -> swerveDrive.getRobotHeading().getRadians(), null);
            
        builder.addStringProperty("rotationUnit",
            () -> "radians", null);

        // Max Speed (in meters per second)
        builder.addDoubleProperty("maxSpeed",
            () -> Constants.SwerveConstants.maxSpeed, null);


        // Independent Controls
        builder.addBooleanProperty("Field Relative",
            () -> swerveDrive.isFieldRelative(),
            (value) -> swerveDrive.setFieldRelative(value));
    }

    @Override
    public void updateTelemetry() {
        // Add any periodic telemetry updates here
    }
}