package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
// import com.revrobotics.spark.config.MAXMotionConfig;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SimpleElevatorConstants;

public class CoralGrabberArm extends SubsystemBase {
    // Hardware
    private final SparkMax armMotor;
    private final RelativeEncoder encoder;
    private final SparkClosedLoopController controller;

    // Position tracking
    private double targetPosition = 0.0;
    private static final double MAX_POSITION = 5.0; // Maximum rotations
    private static final double MIN_POSITION = 0.0;
    private static final double POSITION_CONVERSION_FACTOR = 1.0; // Adjust based on gear ratio
    private static final double MAX_VELOCITY = 2.0; // Rotations per second
    private static final double MAX_ACCELERATION = 1.5; // Rotations per second squared
    
    public CoralGrabberArm() {
        armMotor = new SparkMax(SimpleElevatorConstants.MOTOR_ID, MotorType.kBrushless);
        encoder = armMotor.getEncoder();
        controller = armMotor.getClosedLoopController();
        
        configureMotor();
        encoder.setPosition(0); // Reset encoder on startup
    }

    private void configureMotor() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(true);
        config.smartCurrentLimit(SimpleElevatorConstants.CURRENT_LIMIT);
        config.voltageCompensation(12.0);
        config.idleMode(IdleMode.kBrake);

        // Configure encoder
        config.encoder.positionConversionFactor(POSITION_CONVERSION_FACTOR);
        config.encoder.velocityConversionFactor(POSITION_CONVERSION_FACTOR / 60.0);

        // Configure closed loop
        config.closedLoop
            .p(0.05)
            .i(0.0)
            .d(0.0)
            .outputRange(-1, 1);

        // Configure MAXMotion
        config.closedLoop.maxMotion
            .maxVelocity(MAX_VELOCITY)
            .maxAcceleration(MAX_ACCELERATION)
            .allowedClosedLoopError(1);

        armMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void setSpeed(double speed) {
        // Use speed input to adjust target position
        targetPosition += speed * 0.02; // Scale by loop period (20ms)
        
        // Clamp target position
        targetPosition = Math.min(Math.max(targetPosition, MIN_POSITION), MAX_POSITION);
        
        // Set position using MAXMotion
        controller.setReference(targetPosition, ControlType.kMAXMotionPositionControl);
    }
    
    public void stop() {
        // Maintain current position when stopped
        // targetPosition = encoder.getPosition();
        // controller.setReference(targetPosition, ControlType.kSmartMotion);
    }
    
    @Override
    public void periodic() {
        // Optional: Add position monitoring/logging
    }
    
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("CoralGrabberArm");
        builder.addDoubleProperty("Current Position", encoder::getPosition, null);
        builder.addDoubleProperty("Target Position", () -> targetPosition, null);
        builder.addDoubleProperty("Output", armMotor::get, null);
    }
}
