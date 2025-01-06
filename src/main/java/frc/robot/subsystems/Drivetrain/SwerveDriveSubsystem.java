package frc.robot.subsystems.Drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
// import frc.robot.subsystems.Drivetrain.SwerveDriveSubsystem.CalibrationState;

import java.util.Arrays;

import com.kauailabs.navx.frc.AHRS;

/**
 * The SwerveDriveSubsystem implements a swerve drive train control system.
 * This subsystem manages four swerve modules and handles both robot-centric
 * and field-centric driving modes.
 * 
 * <p>Features include:
 * <ul>
 *   <li>Field-oriented and robot-oriented driving modes</li>
 *   <li>Integrated gyro for field orientation</li>
 *   <li>Debug telemetry publishing</li>
 *   <li>Module state monitoring and control</li>
 * </ul>
 */
public class SwerveDriveSubsystem extends SubsystemBase {
    // Hardware & Configuration
    private final Mk4iSwerveModule[] swerveModules;
    private final AHRS gyro;
    private final SwerveDriveKinematics swerveDriveKinematics;
    
    // State Tracking
    private DriveMode currentMode = DriveMode.NORMAL;
    private boolean isFieldOriented = true;
    private boolean isDebugMode = false;
    // Calibration state

    private boolean isCalibrating = false;
    private int calibrationRetryCount = 0;
    private static final int MAX_CALIBRATION_RETRIES = 3;
    private CalibrationState systemCalibrationState = CalibrationState.IDLE;
    
    // Odometry & Position Tracking
    private final SwerveDriveOdometry odometry;
    
    // Telemetry
     private final StructArrayPublisher<SwerveModuleState> currentStatesPublisher;
    private final StructArrayPublisher<SwerveModuleState> targetStatesPublisher;
    private final SwerveModuleState[] currentStates;
    private final SwerveModuleState[] targetStates;

    /**
     * Constructs a new SwerveDriveSubsystem.
     * Initializes all swerve modules, gyro, and telemetry publishers.
     */
    public SwerveDriveSubsystem() {
        this.swerveDriveKinematics = Constants.SwerveDriveConstants.kinematics;
        this.swerveModules = initializeSwerveModules();
        this.gyro = new AHRS(SPI.Port.kMXP);
        // Initialize odometry
        odometry = new SwerveDriveOdometry(
            swerveDriveKinematics,
            getGyroAngle(),
            getModulePositions()
        );
        // Initial calibration
        calibrateModules();
        // Initialize telemetry arrays and publishers
        this.currentStates = new SwerveModuleState[4];
        this.targetStates = new SwerveModuleState[4];
        this.currentStatesPublisher = createPublisher(Constants.TelemetryConstants.SwerveTopicNames.CURRENT_STATES_TOPIC);
        this.targetStatesPublisher = createPublisher(Constants.TelemetryConstants.SwerveTopicNames.TARGET_STATES_TOPIC);
        resetGyro();
    }
    
    // Drive Control Methods

    /**
     * Sets the drive mode and applies appropriate configurations
     */
    public void setDriveMode(DriveMode mode) {
        if (mode == currentMode) return;
        
        // Handle mode-specific initialization
        switch (mode) {
            case X_LOCK:
                if (currentMode != DriveMode.X_LOCK) {
                    setModulesToXLock();
                }
                break;
            case AUTONOMOUS:
                resetOdometry(new Pose2d()); // Reset to starting position
                break;
            default:
                // Reset to normal driving configuration
                break;
        }
        
        currentMode = mode;
    }

    private void setModulesToXLock() {
        SwerveModuleState[] xLockStates = new SwerveModuleState[4];
        applyModeAdjustments(xLockStates, DriveMode.X_LOCK);
        for (int i = 0; i < swerveModules.length; i++) {
            swerveModules[i].setDesiredState(xLockStates[i]);
        }
    }


    private void applyModeAdjustments(SwerveModuleState[] states, DriveMode mode) {
        if (states == null || states.length != swerveModules.length) {
            throw new IllegalArgumentException("Invalid states array");
        }
        switch (mode) {
            case PRECISION:
                for (SwerveModuleState state : states) {
                    state.speedMetersPerSecond *= Constants.SwerveDriveConstants.kPrecisionModeSpeedMultiplier;
                }
                break;
            case TURBO:
                for (SwerveModuleState state : states) {
                    state.speedMetersPerSecond = Math.min(
                        state.speedMetersPerSecond * Constants.SwerveDriveConstants.kTurboModeSpeedMultiplier,
                        Constants.SwerveDriveConstants.kMaxSpeed
                    );
                }
                break;
            case X_LOCK:
                for (int i = 0; i < states.length; i++) {
                    states[i] = new SwerveModuleState(0, getXLockAngle());
                }
                break;
            default:
                // No adjustments needed
                break;
        }
    }
    private Rotation2d getXLockAngle() {
        // Return appropriate angle based on module position (FL, FR, BL, BR)
        // The zero angle is the front of the robot so the X pattern is 45 degrees
        // since it is the same for all modules, we can hardcode it
        return new Rotation2d(Math.PI / 4);
    }

    /**
     * Commands the swerve drive to move according to the specified chassis speeds.
     * 
     * @param chassisSpeeds The desired chassis speeds (x, y, and rotational velocity)
     */
    public void drive(ChassisSpeeds chassisSpeeds) {
        if (isCalibrating) return;
        
        SwerveModuleState[] states = swerveDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.SwerveDriveConstants.kMaxSpeed);
        
        applyModeAdjustments(states, currentMode);
        
        for (int i = 0; i < swerveModules.length; i++) {
            // Remove mode parameter since it's handled at subsystem level
            swerveModules[i].setDesiredState(states[i]);
        }
    }

    // Mode Management Methods

    
    // Calibration Methods

    /**
     * Attempts to calibrate all swerve modules
     * @return true if all modules calibrated successfully
     */
    public boolean calibrateModules() {
        if (isCalibrating) return false;
        systemCalibrationState = CalibrationState.IN_PROGRESS;
        isCalibrating = true;
        boolean allSuccessful = true;
        for (int i = 0; i < swerveModules.length; i++) {
            try {
                if (!swerveModules[i].calibrate()) {
                    allSuccessful = false;
                }
            } catch (Exception e) {
                allSuccessful = false;
            }
        }
        
        systemCalibrationState = allSuccessful ? CalibrationState.SUCCESS : CalibrationState.FAILED;
        isCalibrating = false;
        
        if (!allSuccessful && calibrationRetryCount < MAX_CALIBRATION_RETRIES) {
            calibrationRetryCount++;
            Timer.delay(0.1);
            return calibrateModules();
        }
        calibrationRetryCount = 0;
        return allSuccessful;
    }
    
    // Telemetry Methods

    
    // Utility Methods

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(getGyroAngle(), getModulePositions(), pose);
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.SwerveDriveConstants.kMaxSpeed);
        for (int i = 0; i < swerveModules.length; i++) {
            swerveModules[i].setDesiredState(desiredStates[i]);
        }
    }

    private SwerveModulePosition[] getModulePositions() {
        return Arrays.stream(swerveModules)
            .map(module -> module.getPosition())
            .toArray(SwerveModulePosition[]::new);
    }

    /**
     * Sets the drive mode between field-oriented and robot-oriented control.
     * 
     * @param fieldOriented true for field-oriented control, false for robot-oriented
     */
    public void setFieldOriented(boolean fieldOriented) {
        isFieldOriented = fieldOriented;
    }

    /**
     * @return current field orientation state
     */
    public boolean isFieldOriented() {
        return isFieldOriented;
    }

    /**
     * Enables or disables debug telemetry publishing.
     * 
     * @param debugMode true to enable debug telemetry, false to disable
     */
    public void setDebugMode(boolean debugMode) {
        isDebugMode = debugMode;
    }

    /**
     * @return current debug mode state
     */
    public boolean isDebugMode() {
        return isDebugMode;
    }

    /**
     * Gets the current gyro angle.
     * 
     * @return The current gyro angle as a Rotation2d
     */
    public Rotation2d getGyroAngle() {
        return Rotation2d.fromDegrees(-gyro.getAngle());
    }

    /**
     * Resets the gyro to zero degrees.
     */
    public void resetGyro() {
        gyro.reset();
    }

    @Override
    public void periodic() {
        // Update odometry
        odometry.update(getGyroAngle(), getModulePositions());
        
        // Periodic calibration check
        if (currentMode != DriveMode.AUTONOMOUS && 
            currentMode != DriveMode.X_LOCK && 
            !isCalibrating) {
            checkModuleCalibration();
        }
        
        if (isDebugMode) {
            updateTelemetry();
        }
    }
    private void checkModuleCalibration() {
        for (Mk4iSwerveModule module : swerveModules) {
            if (module.getCalibrationState() != ModuleCalibrationState.CALIBRATED) {
                calibrateModules();
                break;
            }
        }
    }

    public CalibrationState getSystemCalibrationState() {
        return systemCalibrationState;
    }

    /**
     * Initializes all swerve modules with their respective constants.
     * 
     * @return Array of initialized swerve modules
     */
    private Mk4iSwerveModule[] initializeSwerveModules() {
        return new Mk4iSwerveModule[] {
            new Mk4iSwerveModule(Constants.SwerveDriveConstants.frontLeftModuleConstants),
            new Mk4iSwerveModule(Constants.SwerveDriveConstants.frontRightModuleConstants),
            new Mk4iSwerveModule(Constants.SwerveDriveConstants.backLeftModuleConstants),
            new Mk4iSwerveModule(Constants.SwerveDriveConstants.backRightModuleConstants)
        };
    }

    /**
     * Creates a NetworkTables publisher for swerve module states.
     * 
     * @param topic The NetworkTables topic name
     * @return Configured StructArrayPublisher
     */
    private StructArrayPublisher<SwerveModuleState> createPublisher(String topic) {
        return NetworkTableInstance.getDefault()
            .getStructArrayTopic(topic, SwerveModuleState.struct)
            .publish();
    }

    /**
     * Updates the current and target states arrays with latest module data.
     */
    private void updateModuleStates() {
        for (int i = 0; i < swerveModules.length; i++) {
            currentStates[i] = swerveModules[i].getState();
            targetStates[i] = swerveModules[i].getTargetState();
        }
    }

    /**
     * Publishes current telemetry data to NetworkTables.
     */
    private void publishTelemetry() {
        currentStatesPublisher.set(currentStates);
        targetStatesPublisher.set(targetStates);
    }

    /**
     * Updates telemetry data for all swerve modules.
     */
    private void updateTelemetry() {
        updateModuleStates();
        publishTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        // Add simulation behavior if needed
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addBooleanProperty("Field Oriented", this::isFieldOriented, this::setFieldOriented);
        builder.addBooleanProperty("Debug Mode", this::isDebugMode, this::setDebugMode);
    }
}