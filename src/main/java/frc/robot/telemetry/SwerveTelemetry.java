package frc.robot.telemetry;

import java.util.Map;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveDrivetrain;

public class SwerveTelemetry implements SubsystemTelemetry {
    private final Field2d field2d = new Field2d();
    private final SwerveDrivetrain swerveDrive;
    private boolean detailedLogging = false;

    public SwerveTelemetry(SwerveDrivetrain swerveDrive) {
        this.swerveDrive = swerveDrive;
        initDashboard();
    }

    public void setDetailedLogging(boolean enabled) {
        detailedLogging = enabled;
    }

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
        
        // YAGSL Required Properties
        builder.addDoubleArrayProperty("measuredStates", 
            () -> convertToTelemetryArray(swerveDrive.getCurrentStates()), null);
        builder.addDoubleArrayProperty("desiredStates",
            () -> convertToTelemetryArray(swerveDrive.getTargetStates()), null);
        builder.addDoubleProperty("robotRotation",
            () -> swerveDrive.getRobotHeading().getRadians(), null);
        builder.addStringProperty("rotationUnit",
            () -> "radians", null);
        builder.addDoubleProperty("maxSpeed",
            () -> Constants.SwerveConstants.maxSpeed, null);
            
        // Robot dimensions (in meters) for visualization
        // builder.addDoubleProperty("sizeLeftRight",
        //     () -> Constants.SwerveConstants.trackWidth, null);
        // builder.addDoubleProperty("sizeFrontBack",
        //     () -> Constants.SwerveConstants.wheelBase, null);

        // Control Properties
        builder.addBooleanProperty("Field Relative",
            () -> swerveDrive.isFieldRelative(),
            (value) -> swerveDrive.setFieldRelative(value));
            
        // Detailed logging (when enabled)
        if (detailedLogging) {
            // Module-specific details
            for (int i = 0; i < 4; i++) {
                final int moduleIndex = i;
                String moduleName = getModuleName(i);
                
                builder.addDoubleProperty(
                    moduleName + " Velocity Error",
                    () -> getModuleVelocityError(moduleIndex), 
                    null);
                builder.addDoubleProperty(
                    moduleName + " Angle Error",
                    () -> getModuleAngleError(moduleIndex), 
                    null);
            }
            
            // Odometry Information
            builder.addDoubleProperty("Robot X Position",
                () -> swerveDrive.getPose().getX(), null);
            builder.addDoubleProperty("Robot Y Position",
                () -> swerveDrive.getPose().getY(), null);

            builder.addDoubleProperty("Steering/TargetAngle", 
                ()->swerveDrive.getTargetStates()[0].angle.getRotations(), null);
            builder.addDoubleProperty("Steering/CurrentAngle", 
                ()->swerveDrive.getCurrentStates()[0].angle.getRotations(), null);
        }
    }
    
    private String getModuleName(int index) {
        switch (index) {
            case 0: return "Front Left";
            case 1: return "Front Right";
            case 2: return "Back Left";
            case 3: return "Back Right";
            default: return "Module " + index;
        }
    }

    private double getModuleVelocityError(int moduleIndex) {
        SwerveModuleState current = swerveDrive.getCurrentStates()[moduleIndex];
        SwerveModuleState target = swerveDrive.getTargetStates()[moduleIndex];
        return target.speedMetersPerSecond - current.speedMetersPerSecond;
    }

    private double getModuleAngleError(int moduleIndex) {
        SwerveModuleState current = swerveDrive.getCurrentStates()[moduleIndex];
        SwerveModuleState target = swerveDrive.getTargetStates()[moduleIndex];
        return target.angle.minus(current.angle).getRadians();
    }

    @Override
    public void updateTelemetry() {
        // Update robot position on field
        field2d.setRobotPose(swerveDrive.getPose());

        // Update trajectory visualization
        String activeTrajectory = swerveDrive.getActiveTrajectoryName();
        if (!activeTrajectory.isEmpty()) {
            field2d.getObject("Active-Trajectory")
                .setTrajectory(swerveDrive.getTrajectories().get(activeTrajectory));
        } else {
            field2d.getObject("Active-Trajectory").setPoses();
        }

        // Show all stored trajectories
        for (Map.Entry<String, Trajectory> entry : swerveDrive.getTrajectories().entrySet()) {
            field2d.getObject("Trajectory-" + entry.getKey())
                .setTrajectory(entry.getValue());
        }
    }

    public void initDashboard() {
        SmartDashboard.putData("Field", field2d);
    }
}