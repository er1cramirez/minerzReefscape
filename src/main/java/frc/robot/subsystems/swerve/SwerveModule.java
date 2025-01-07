package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.Mk4iModuleConstants;
import frc.robot.util.SwerveModuleOperationState;

public class SwerveModule extends SubsystemBase{

    // Constants

    // Hardware
    // * Motors
    private final SparkMax drivingMotor;
    private final SparkMax steeringMotor;
    // * Encoders
    private final RelativeEncoder drivingEncoder;
    private final RelativeEncoder steeringEncoder;
    private final CANcoder steerAbsoluteEncoder;

    // Controllers
    private final SparkClosedLoopController drivingController;
    private final SparkClosedLoopController steeringController;

    // Configuration
    private final Mk4iModuleConstants moduleConstants;

    // State variables
    private SwerveModuleOperationState operationState;
    private SwerveModuleState currentState;
    private SwerveModuleState targetState;

    // Constructor

    /**
     * Constructs a SwerveModule object.
     * 
     * @param moduleConstants The constants for the module
     */
    public SwerveModule(Mk4iModuleConstants moduleConstants){
        try{
            this.moduleConstants = moduleConstants;

            // Initialize hardware
            drivingMotor = new SparkMax(moduleConstants.drivingMotorID(), MotorType.kBrushless);
            steeringMotor = new SparkMax(moduleConstants.steeringMotorID(), MotorType.kBrushless);
            drivingEncoder = drivingMotor.getEncoder();
            steeringEncoder = steeringMotor.getEncoder();
            steerAbsoluteEncoder = new CANcoder(moduleConstants.steerAbsoluteEncoderID());

            // Initialize controllers
            drivingController = drivingMotor.getClosedLoopController();
            steeringController = steeringMotor.getClosedLoopController();

            // Motors configuration
            drivingMotorInit();
            steeringMotorInit();

            // Initialize state variables
            currentState = new SwerveModuleState(0, getModuleSteer());
            targetState = currentState;
        } catch (Exception e){
            throw new RuntimeException("Module " + moduleConstants.moduleNumber() + " failed to initialize", e);
        }

    }

    public void setDesiredState(SwerveModuleState desiredState){
        targetState = desiredState;
        updateCurrentState();
        // Optimize the reference state to avoid spinning the module 180 degrees
        desiredState.optimize(currentState.angle);
        // Cosine scaling for the driving motor
        desiredState.cosineScale(currentState.angle);

        drivingController.setReference(
            desiredState.speedMetersPerSecond, 
            ControlType.kVelocity
        );
        steeringController.setReference(
            desiredState.angle.getRadians(), 
            ControlType.kPosition
        );
    }
    // Configuration methods

    private void drivingMotorInit(){
    }

    private void steeringMotorInit(){
        resetSteeringEncoderToAbsolute();
    }


    // State methods

    private void updateCurrentState() {
        double velocity = drivingEncoder.getVelocity();
        currentState = new SwerveModuleState(velocity, getModuleSteer());
    }


    private double getModuleSpeed(){
        return drivingEncoder.getVelocity();
    }

    private Rotation2d getModuleSteer(){
        return new Rotation2d(steeringEncoder.getPosition());
    }

    // CANCoder methods
    /*
     * public StatusSignal<Angle> getAbsolutePosition()
        Absolute Position of the device. The possible range is documented below; however, the exact expected range is determined by the AbsoluteSensorDiscontinuityPoint. This position is only affected by the MagnetSensor configs.
        Minimum Value: -1.0
        Maximum Value: 0.999755859375
        Default Value: 0
        Units: rotations
     */
    private Rotation2d getAbsoluteSteer(){
        return Rotation2d.fromRotations(steerAbsoluteEncoder.getAbsolutePosition().getValueAsDouble()).minus(moduleConstants.steerAngleOffset());
    }

    /**
     * Resets the steering encoder to match the absolute encoder position.
     * Includes rate limiting to prevent excessive resets.
     */
    private void resetSteeringEncoderToAbsolute() {
        operationState = SwerveModuleOperationState.CALIBRATING; // Add state tracking
        try {
            Rotation2d position = getAbsoluteSteer();
            steeringEncoder.setPosition(position.getRadians());
            operationState = SwerveModuleOperationState.CALIBRATED;
        } catch (Exception e) {
            operationState = SwerveModuleOperationState.FAILED;
            throw e;
        }
    } 

    private void checkRunningState() {
        if (Math.abs(getModuleSpeed()) < Constants.SwerveConstants.speedDeadband) {
            operationState = SwerveModuleOperationState.IDLE;
        } else {
            operationState = SwerveModuleOperationState.RUNNING;
        }
    }


    public SwerveModuleOperationState getCalibrationState() {
        // Only check if we think we're calibrated
        if (operationState == SwerveModuleOperationState.CALIBRATED) {
            try {
                // Get the absolute difference between angles
                double angleDifference = Math.abs(getAbsoluteSteer().minus(getAbsoluteSteer()).getRadians()
                );
                // Check if difference exceeds tolerance
                if (angleDifference > Constants.SwerveConstants.steeringErrorTolerance) {
                    operationState = SwerveModuleOperationState.UNCALIBRATED;
                }
            } catch (Exception e) {
                // Handle any sensor reading errors
                operationState = SwerveModuleOperationState.FAILED;
            }
        }
        return operationState;
    }

    public boolean calibrateSteering() {
        checkRunningState();
        if (operationState == SwerveModuleOperationState.IDLE) {
            if (getCalibrationState() != SwerveModuleOperationState.CALIBRATED) {
                try {
                    resetSteeringEncoderToAbsolute();
                    return true;
                } catch (Exception e) {
                    operationState = SwerveModuleOperationState.FAILED;
                    return false;
                }
            }
        }
        return false;
    }

    public SwerveModuleState getCurrentState() {
        return currentState;
    }

    /**
     * Gets the desired state of the module
     */
    public SwerveModuleState getTargetState() {
        return targetState;
    }
        
}
