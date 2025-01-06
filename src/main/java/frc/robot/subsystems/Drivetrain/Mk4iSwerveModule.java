package frc.robot.subsystems.Drivetrain;
// WPILib imports
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
// REV imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
// CTRE imports
import com.ctre.phoenix6.hardware.CANcoder;
// Project imports
import frc.robot.constants.Constants;
import frc.robot.constants.Mk4iSwerveModuleConstants;
import frc.robot.constants.PIDConstants;

/**
 * Implementation of a MK4i Swerve Module using REV SparkMax controllers and a CTRE CANCoder.
 * Features include:
 * 
 * <ul>
 *   <li>Automatic encoder calibration</li>
 *   <li>State monitoring and fault detection</li>
 *   <li>Voltage compensation</li>
 * </ul>
 */
public class Mk4iSwerveModule {
    // Constants (at top)
    private static final double MIN_CALIBRATION_INTERVAL = 0.1;
    private static final double VELOCITY_TOLERANCE = 0.1;
    
    // Hardware Components
    private final CANSparkMax drivingMotor;
    private final CANSparkMax steeringMotor;
    private final RelativeEncoder drivingEncoder;
    private final RelativeEncoder steeringEncoder;
    private final CANcoder steerAbsoluteEncoder;
    
    // Controllers
    private final SparkPIDController drivingController;
    private final SparkPIDController steeringController;
    
    // Configuration
    private final Mk4iSwerveModuleConstants moduleConstants;
    private final Rotation2d chassisAngularOffset;
    private final int moduleNumber;
    
    // State Tracking
    private SwerveModuleState targetState;
    private ModuleCalibrationState calibrationState = ModuleCalibrationState.UNCALIBRATED;
    private double lastCalibrationTime = 0;
    
    // Constructor & Initialization Methods
     /**
     * Creates a new Mk4iSwerveModule.
     *
     * @param moduleConstants Configuration constants for this module
     */
    public Mk4iSwerveModule(Mk4iSwerveModuleConstants moduleConstants) {
        this.moduleConstants = moduleConstants;
        moduleNumber = moduleConstants.moduleNumber();
        // Initialize hardware
        drivingMotor = new CANSparkMax(moduleConstants.drivingMotorID(), MotorType.kBrushless);
        steeringMotor = new CANSparkMax(moduleConstants.drivingMotorID(), MotorType.kBrushless);
        drivingEncoder = drivingMotor.getEncoder();
        steeringEncoder = steeringMotor.getEncoder();
        steerAbsoluteEncoder = new CANcoder(moduleConstants.steerAbsoluteEncoderID());
        // Store configuration
        chassisAngularOffset = moduleConstants.steerAngleOffset();
        // drivingPIDConstants = moduleConstants.drivingPIDConstants();
        // steeringPIDConstants = moduleConstants.steeringPIDConstants();  
        // Initialize controllers
        drivingController = drivingMotor.getPIDController();
        steeringController = steeringMotor.getPIDController();
        // Initialize module
        initializeModule();
    }

    /**
     * Initializes all module components with proper configuration.
     */
    private void initializeModule() {
        try {
            canCoderConfig();
            drivingConfig();
            steeringConfig();
            setDesiredState(getState());
        } catch (Exception e) {
            throw new RuntimeException("Module " + moduleNumber + " initialization failed", e);
        }
    }

    // Public Control Methods

    /**
     * Sets the desired state with mode-specific adjustments
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the state
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle);
        // Apply standard compensations
        desiredState.speedMetersPerSecond *= desiredState.angle.minus(getState().angle).getCos();
        drivingController.setReference(
            desiredState.speedMetersPerSecond, 
            ControlType.kVelocity
        );
        steeringController.setReference(
            desiredState.angle.getRadians(), 
            ControlType.kPosition
        );
        
        targetState = desiredState;
    }
    
    // Calibration Methods

     /**
     * Attempts to calibrate the module by resetting to absolute encoder position.
     * @return true if calibration was successful
     */
    public boolean calibrate() {
        if (!canCalibrate()) {
            return false;
        }
        
        try {
            resetSteeringEncoderToAbsolute();
            calibrationState = ModuleCalibrationState.CALIBRATED;
            lastCalibrationTime = Timer.getFPGATimestamp();
            return true;
        } catch (Exception e) {
            calibrationState = ModuleCalibrationState.FAILED;
            return false;
        }
    }
    
    private boolean canCalibrate() {
        return Timer.getFPGATimestamp() - lastCalibrationTime >= MIN_CALIBRATION_INTERVAL &&
               isModuleReadyForCalibration();
    }
    
    /**
     * Resets the steering encoder to match the absolute encoder position.
     * Includes rate limiting to prevent excessive resets.
     */
    private void resetSteeringEncoderToAbsolute() {
        calibrationState = ModuleCalibrationState.CALIBRATING; // Add state tracking
        try {
            double absolutePosition = steerAbsoluteEncoder.getAbsolutePosition().getValue();
            if (!isValidAbsolutePosition(absolutePosition)) {
                calibrationState = ModuleCalibrationState.FAILED;
                throw new IllegalStateException("Invalid absolute encoder position");
            }
            
            Rotation2d position = Rotation2d.fromRotations(absolutePosition).minus(chassisAngularOffset);
            steeringEncoder.setPosition(position.getRadians());
            calibrationState = ModuleCalibrationState.CALIBRATED;
        } catch (Exception e) {
            calibrationState = ModuleCalibrationState.FAILED;
            throw e;
        }
    } 

    private boolean isModuleReadyForCalibration() {
        return Math.abs(steeringEncoder.getVelocity()) < VELOCITY_TOLERANCE &&
               Math.abs(drivingEncoder.getVelocity()) < VELOCITY_TOLERANCE;
    }

    private boolean isValidAbsolutePosition(double position) {
        return !Double.isNaN(position) && 
               !Double.isInfinite(position) && 
               Math.abs(position) <= 2 * Math.PI;
    }

    public ModuleCalibrationState getCalibrationState() {
        // Only check if we think we're calibrated
        if (calibrationState == ModuleCalibrationState.CALIBRATED) {
            try {
                // Get absolute position
                double rawAbsolute = steerAbsoluteEncoder.getAbsolutePosition().getValue();
                if (!isValidAbsolutePosition(rawAbsolute)) {
                    calibrationState = ModuleCalibrationState.FAILED;
                    return calibrationState;
                }
    
                // Convert both to Rotation2d for proper angle comparison
                Rotation2d absolutePosition = Rotation2d.fromRotations(rawAbsolute)
                                                      .minus(chassisAngularOffset);
                Rotation2d steeringPosition = Rotation2d.fromRadians(steeringEncoder.getPosition());
                
                // Get the absolute difference between angles
                double angleDifference = Math.abs(
                    absolutePosition.minus(steeringPosition).getRadians()
                );
    
                // Check if difference exceeds tolerance
                if (angleDifference > Constants.SwerveDriveConstants.kSteeringErrorTolerance) {
                    calibrationState = ModuleCalibrationState.UNCALIBRATED;
                }
            } catch (Exception e) {
                // Handle any sensor reading errors
                calibrationState = ModuleCalibrationState.FAILED;
            }
        }
        return calibrationState;
    }

    // Configuration Methods

    /**
     * Configures the CANCoder with optimized settings.
     */
    private void canCoderConfig() { }

    /**
     * Configures the driving motor with proper settings and safety limits.
     */
    private void drivingConfig() {
        drivingMotor.restoreFactoryDefaults();
        drivingMotor.setIdleMode(Constants.MotorConstants.driveIdleMode);
        drivingMotor.setSmartCurrentLimit(Constants.MotorConstants.driveCurrentLimit);
        drivingMotor.setInverted(moduleConstants.isDriveMotorInverted());
        // Configure encoder
        double drivingFactor = Constants.SwerveDriveConstants.Mk4iMechanicalConstants.drivingFactor;
        drivingEncoder.setPositionConversionFactor(drivingFactor);
        drivingEncoder.setVelocityConversionFactor(drivingFactor / 60.0);
        drivingEncoder.setPosition(0);
        // Configure PID
        configurePIDController(drivingController, moduleConstants.drivingPIDConstants(), false);
        // Enable voltage compensation
        drivingMotor.enableVoltageCompensation(Constants.MotorConstants.voltageCompensation);
        drivingMotor.burnFlash();
    }

    /**
     * Configures the steering motor with proper settings and safety limits.
     */
    private void steeringConfig() {
        steeringMotor.restoreFactoryDefaults();
        steeringMotor.setIdleMode(Constants.MotorConstants.steerIdleMode);
        steeringMotor.setSmartCurrentLimit(Constants.MotorConstants.steerCurrentLimit);
        steeringMotor.setInverted(moduleConstants.isSteerMotorInverted());
        // Configure encoder
        double steeringFactor = Constants.SwerveDriveConstants.Mk4iMechanicalConstants.steeringFactor;
        steeringEncoder.setPositionConversionFactor(steeringFactor);
        steeringEncoder.setVelocityConversionFactor(steeringFactor / 60.0);
        // Configure PID
        configurePIDController(steeringController, moduleConstants.steeringPIDConstants(), true);
        resetSteeringEncoderToAbsolute();
        steeringMotor.burnFlash();
    }

    /**
     * Configures a PID controller with the specified constants.
     */
    private void configurePIDController(SparkPIDController controller, PIDConstants constants, boolean enableWrapping) {
        controller.setP(constants.kP());
        controller.setI(constants.kI());
        controller.setD(constants.kD());
        controller.setOutputRange(constants.kMinOutput(), constants.kMaxOutput());
        if (enableWrapping) {
            controller.setPositionPIDWrappingEnabled(true);
            controller.setPositionPIDWrappingMinInput(-Math.PI);
            controller.setPositionPIDWrappingMaxInput(Math.PI);
        }
    }

    // State & Telemetry Methods

    public SwerveModuleState getState() {
        return new SwerveModuleState(
            drivingEncoder.getVelocity(),
            Rotation2d.fromRadians(steeringEncoder.getPosition())
        );
    }

    public SwerveModuleState getTargetState() {
        return targetState;
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            drivingEncoder.getPosition(),
            Rotation2d.fromRadians(steeringEncoder.getPosition())
        );
    }
    // Private Utility Methods
}