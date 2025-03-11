package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SimpleElevatorConstants;

public class CoralGrabberArm extends SubsystemBase {
    // Hardware
    private final SparkMax armMotor;
    private final RelativeEncoder encoder;
    private final AbsoluteEncoder absEncoder;
    private final SparkClosedLoopController controller;

    // Position tracking
    private double targetPosition = 0.0;
    private static final double GEAR_RATIO = 36; // Verify actual gear ratio
    private static final double DEADBAND = 0.05;

    // Motion profiling
    private final TrapezoidProfile.Constraints constraints;
    private final ProfiledPIDController profiledController;
    private final double MAX_VELOCITY = 0.8; // rotations per second
    private final double MAX_ACCELERATION = 0.5; // rotations per second²
    
    // Control mode tracking
    private boolean wasMoving = false;
    private boolean isProfileControlActive = false;
    
    // Pre-defined positions (can be moved to Constants)
    public static final double STOWED_POSITION = 0.0;
    public static final double SCORING_POSITION = Math.PI/2; // Adjust based on your mechanism
    
    public CoralGrabberArm() {
        armMotor = new SparkMax(SimpleElevatorConstants.MOTOR_ID, MotorType.kBrushless);
        encoder = armMotor.getEncoder();
        absEncoder = armMotor.getAbsoluteEncoder();
        controller = armMotor.getClosedLoopController();
        
        constraints = new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION);
        profiledController = new ProfiledPIDController(5.0, 0.0, 0.5, constraints);
        profiledController.setTolerance(Math.toRadians(2), 0.05);
        
        configureMotor();

        SmartDashboard.putData("CoralGrabberArm", this);
    }

    private void configureMotor() {
        // Same motor configuration as before
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(true);
        config.smartCurrentLimit(20);
        config.voltageCompensation(12.0);
        config.idleMode(IdleMode.kBrake);

        // Configure relative encoder
        config.encoder.positionConversionFactor(1.0 / GEAR_RATIO);
        config.encoder.velocityConversionFactor(1.0 / (GEAR_RATIO * 60.0));

        // Configure absolute encoder
        config.absoluteEncoder.positionConversionFactor(2.0 * Math.PI);
        config.absoluteEncoder.velocityConversionFactor((2.0 * Math.PI) / 60.0);
        config.absoluteEncoder.averageDepth(2);
        
        // Configure closed loop for velocity control
        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(0.1)
            .i(0.0)
            .d(0.05)
            .velocityFF(0.2)
            .outputRange(-1, 1);

        armMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        
        // Initialize position based on absolute encoder
        double absPosition = absEncoder.getPosition();
        encoder.setPosition(absPosition);
    }

    /**
     * Set arm position using the onboard PID controller
     */
    public void setPositionDirectPID(double position) {
        isProfileControlActive = false;
        targetPosition = position;
        controller.setReference(position, ControlType.kPosition);
    }
    
    /**
     * Creates a command that moves the arm to a target position using profiled PID
     */
    public Command createMoveToPositionCommand(double position) {
        return Commands.sequence(
            // Initialize movement
            Commands.runOnce(() -> {
                targetPosition = position;
                profiledController.reset(absEncoder.getPosition(), absEncoder.getVelocity());
                profiledController.setGoal(position);
                isProfileControlActive = true;
            }),
            
            // Continue execution until at setpoint
            Commands.run(this::executeProfiledControl, this)
                .until(profiledController::atGoal),
            
            // Finalize with direct position control
            Commands.runOnce(() -> {
                setPositionDirectPID(position);
            })
        ).withName("MoveArmTo" + position);
    }
    
    /**
     * Move arm to stowed position
     */
    public Command stowCommand() {
        return createMoveToPositionCommand(STOWED_POSITION).withName("StowArm");
    }
    
    /**
     * Move arm to scoring position
     */
    public Command scoreCommand() {
        return createMoveToPositionCommand(SCORING_POSITION).withName("ScoreArm");
    }
    
    /**
     * Execute one cycle of the profiled PID control
     */
    private void executeProfiledControl() {
        // Get current actual position from absolute encoder
        double currentPosition = absEncoder.getPosition();
        // double velocity = absEncoder.getVelocity();
        
        // Calculate next velocity using the profiled PID controller
        double output = profiledController.calculate(currentPosition);
        
        // Use velocity control mode with the calculated output
        controller.setReference(output, ControlType.kVelocity);
    }

    /**
     * Manual control via joystick input
     */
    public void setSpeed(double speed) {
        if (Math.abs(speed) > DEADBAND) {
            // Direct speed control when there's input
            isProfileControlActive = false;
            armMotor.set(speed);
            wasMoving = true;
        } else {
            if (wasMoving) {
                // Capture position when input stops
                targetPosition = absEncoder.getPosition();
                setPositionDirectPID(targetPosition);
                wasMoving = false;
            }
        }
    }
    
    /**
     * Stop the arm and hold position
     */
    public void stop() {
        targetPosition = absEncoder.getPosition();
        setPositionDirectPID(targetPosition);
    }

    /**
     * Get current arm position
     */
    public double getPosition() {
        return absEncoder.getPosition();
    }
    
    @Override
    public void periodic() {
        // Simplified periodic - just update telemetry
    }
    
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("CoralGrabberArm");
        builder.addDoubleProperty("Current Position", absEncoder::getPosition, null);
        builder.addDoubleProperty("Target Position", () -> targetPosition, null);
        builder.addDoubleProperty("Velocity", absEncoder::getVelocity, null);
        builder.addBooleanProperty("At Goal", () -> !isProfileControlActive || profiledController.atGoal(), null);
    }
}
