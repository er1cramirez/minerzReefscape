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
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SimpleElevatorConstants;

public class CoralGrabberArm extends SubsystemBase {
    // Hardware
    private final SparkMax armMotor;
    private final RelativeEncoder encoder;
    private final SparkClosedLoopController controller;
    private final DigitalInput limitSwitch; // Home position limit switch
    private final Debouncer limitSwitchDebouncer;

    // Position tracking
    private double targetPosition = 0.0;
    private static final double GEAR_RATIO = 36; // Verify actual gear ratio
    private static final double DEADBAND = 0.05;
    private static final int LIMIT_SWITCH_PORT = 4; // Set to actual port
    private boolean isHomedFlag = false;

    // Motion profiling
    private final TrapezoidProfile.Constraints constraints;
    private final ProfiledPIDController profiledController;
    private final double MAX_VELOCITY = 0.8; // rotations per second
    private final double MAX_ACCELERATION = 0.5; // rotations per second²
    
    // Control mode tracking
    private boolean wasMoving = false;
    private boolean isProfileControlActive = false;
    
    // Homing constants
    private static final double HOMING_SPEED = -0.08; // Slow speed for finding limit switch
    private static final double DEBOUNCE_TIME = 0.1; // 100ms debounce for limit switch
    
    // Pre-defined positions (can be moved to Constants)
    public static final double STOWED_POSITION = 0.0;
    public static final double SCORING_POSITION = Math.PI/2; // Adjust based on your mechanism
    
    public CoralGrabberArm() {
        armMotor = new SparkMax(SimpleElevatorConstants.MOTOR_ID, MotorType.kBrushless);
        encoder = armMotor.getEncoder();
        controller = armMotor.getClosedLoopController();
        limitSwitch = new DigitalInput(LIMIT_SWITCH_PORT);
        limitSwitchDebouncer = new Debouncer(DEBOUNCE_TIME, DebounceType.kBoth);
        
        constraints = new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION);
        
        // Configure the ProfiledPIDController for high-level position control
        profiledController = new ProfiledPIDController(1.5, 0.0, 0.1, constraints);
        profiledController.setTolerance(Math.toRadians(2), 0.05);
        
        configureMotor();

        SmartDashboard.putData("CoralGrabberArm", this);
    }

    private void configureMotor() {
        SparkMaxConfig config = new SparkMaxConfig();
        // Motor configuration
        config.inverted(true);
        config.smartCurrentLimit(20);
        config.voltageCompensation(12.0);
        config.idleMode(IdleMode.kBrake);

        // Configure relative encoder for both position and velocity control
        config.encoder.positionConversionFactor((2.0 * Math.PI) / GEAR_RATIO);
        config.encoder.velocityConversionFactor((2.0 * Math.PI) / (GEAR_RATIO * 60.0));
        
        // Configure SparkMax PID for velocity control only
        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(0.1)     // Velocity control P
            .i(0.0)     // I term
            .d(0.0)     // D term for velocity
            .outputRange(-1, 1);

        armMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    
    /**
     * Creates a homing command that sets the zero position based on the limit switch
     */
    public Command homeCommand() {
        return Commands.sequence(
            // Check if we're already at home position
            Commands.either(
                // If already at home, just reset encoder
                Commands.runOnce(() -> {
                    encoder.setPosition(0);
                    isHomedFlag = true;
                    targetPosition = 0;
                    profiledController.reset(0, 0);
                }),
                
                // Otherwise run the full homing sequence
                Commands.sequence(
                    // Start homing - move towards limit switch at slow speed
                    Commands.runOnce(() -> {
                        isHomedFlag = false;
                        armMotor.set(HOMING_SPEED);
                    }),
                    
                    // Wait until limit switch is triggered
                    Commands.waitUntil(() -> getDebouncedLimitSwitch()),
                    
                    // Stop motor and reset encoder
                    Commands.runOnce(() -> {
                        armMotor.set(0);
                        encoder.setPosition(0);
                        isHomedFlag = true;
                        targetPosition = 0;
                        profiledController.reset(0, 0);
                    }),
                    
                    // Move slightly away from limit switch to avoid triggering again
                    createMoveToPositionCommand(Math.toRadians(5))
                ),
                
                // Condition for whether we're already at home
                this::isAtLimitSwitch
            )
        ).withName("HomeArm");
    }

    /**
     * Checks if the limit switch is currently triggered (with debounce)
     */
    private boolean getDebouncedLimitSwitch() {
        return limitSwitchDebouncer.calculate(!limitSwitch.get());
    }
    
    /**
     * Checks if the arm is currently at the limit switch position
     */
    private boolean isAtLimitSwitch() {
        return getDebouncedLimitSwitch();
    }

    /**
     * Creates a command that moves the arm to a target position using profiled PID
     */
    public Command createMoveToPositionCommand(double position) {
        return Commands.sequence(
            // Cannot move if not homed - home first if necessary
            Commands.either(
                Commands.none(),
                homeCommand(),
                () -> isHomed()
            ),
            
            // Initialize movement
            Commands.runOnce(() -> {
                targetPosition = position;
                profiledController.reset(encoder.getPosition(), encoder.getVelocity());
                profiledController.setGoal(position);
                isProfileControlActive = true;
            }),
            
            // Continue execution until at setpoint
            Commands.run(this::executeProfiledControl, this)
                .until(() -> profiledController.atGoal()),
            
            // Finalize by maintaining the velocity at zero
            Commands.runOnce(() -> {
                controller.setReference(0, ControlType.kVelocity);
                isProfileControlActive = false;
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
     * This is called by the command during profiled movement
     */
    private void executeProfiledControl() {
        // Use relative encoder for position feedback
        double currentPosition = encoder.getPosition();
        
        // Calculate desired velocity using the profiled PID controller
        double velocityOutput = profiledController.calculate(currentPosition);
        
        // Send velocity command to SparkMax
        controller.setReference(velocityOutput, ControlType.kVelocity);
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
        } else if (wasMoving) {
            // When joystick is released, hold position using profiled control
            targetPosition = encoder.getPosition();
            profiledController.reset(targetPosition, encoder.getVelocity());
            profiledController.setGoal(targetPosition);
            isProfileControlActive = true;
            wasMoving = false;
        } else if (isProfileControlActive) {
            // Continue using profiled control
            executeProfiledControl();
        } else {
            // Maintain zero velocity
            controller.setReference(0, ControlType.kVelocity);
        }
    }
    
    /**
     * Stop the arm and hold current position
     */
    public void stop() {
        targetPosition = encoder.getPosition();
        profiledController.reset(targetPosition, encoder.getVelocity());
        profiledController.setGoal(targetPosition);
        isProfileControlActive = true;
        wasMoving = false;
    }

    /**
     * Get current arm position
     */
    public double getPosition() {
        return encoder.getPosition();
    }
    
    /**
     * Check if the arm is currently homed
     * This considers both our tracked home state and physical limit switch position
     */
    public boolean isHomed() {
        return isHomedFlag || isAtLimitSwitch();
    }
    
    /**
     * Reset the arm position to zero at the current position
     * This should only be used for testing/calibration
     */
    public void resetPosition() {
        encoder.setPosition(0);
        targetPosition = 0;
        profiledController.reset(0, 0);
        isHomedFlag = true;
    }
    
    @Override
    public void periodic() {
    }
    
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("CoralGrabberArm");
        builder.addDoubleProperty("Current Position", encoder::getPosition, null);
        builder.addDoubleProperty("Target Position", () -> targetPosition, null);
    }
}
