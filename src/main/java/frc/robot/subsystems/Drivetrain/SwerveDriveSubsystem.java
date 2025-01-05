package frc.robot.subsystems.Drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
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
    // NetworkTables topics for telemetry
    private static final String CURRENT_STATES_TOPIC = "/SwerveStates/Current";
    private static final String TARGET_STATES_TOPIC = "/SwerveStates/Target";
    
    // Hardware components
    private final Mk4iSwerveModule[] swerveModules;
    private final AHRS gyro;
    private final SwerveDriveKinematics swerveDriveKinematics;

    // State tracking
    private boolean isFieldOriented = true;
    private boolean isDebugMode = false;

    // Telemetry publishers
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
        
        // Initialize telemetry arrays and publishers
        this.currentStates = new SwerveModuleState[4];
        this.targetStates = new SwerveModuleState[4];
        this.currentStatesPublisher = createPublisher(CURRENT_STATES_TOPIC);
        this.targetStatesPublisher = createPublisher(TARGET_STATES_TOPIC);
        
        resetGyro();
    }

    /**
     * Commands the swerve drive to move according to the specified chassis speeds.
     * 
     * @param chassisSpeeds The desired chassis speeds (x, y, and rotational velocity)
     */
    public void drive(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] desiredStates = swerveDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.SwerveDriveConstants.kMaxSpeed);
        setDesiredStates(desiredStates);
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
        if (isDebugMode) {
            updateModuleStates();
            publishTelemetry();
        }
    }

    /**
     * Sets the desired states for all swerve modules.
     * 
     * @param desiredStates Array of desired states for each module
     * @throws IllegalArgumentException if desiredStates length doesn't match module count
     */
    public void setDesiredStates(SwerveModuleState[] desiredStates) {
        if (desiredStates.length != swerveModules.length) {
            throw new IllegalArgumentException("Desired states array length must match number of modules");
        }
        
        for (int i = 0; i < swerveModules.length; i++) {
            swerveModules[i].setDesiredState(desiredStates[i]);
        }
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