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
// import com.revrobotics.spark.config.MAXMotionConfig;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SimpleElevatorConstants;
// import edu.wpi.first.math.MathUtil;

public class CoralGrabberArm extends SubsystemBase {
    // Hardware
    private final SparkMax armMotor;
    private final RelativeEncoder encoder;
    private final AbsoluteEncoder absEncoder;
    private final SparkClosedLoopController controller;

    // Position tracking
    private double targetPosition = 0.0;
    // private static final double MAX_POSITION = 5; // Adjust these limits based on mechanism
    // private static final double MIN_POSITION = -5;
    private static final double GEAR_RATIO = 36; // Verify actual gear ratio
    private static final double DEADBAND = 0.05;

    private boolean wasMoving = false;
    
    public CoralGrabberArm() {
        armMotor = new SparkMax(SimpleElevatorConstants.MOTOR_ID, MotorType.kBrushless);
        encoder = armMotor.getEncoder();
        absEncoder = armMotor.getAbsoluteEncoder();
        controller = armMotor.getClosedLoopController();
        
        configureMotor();
        encoder.setPosition(0); // Reset encoder on startup

        SmartDashboard.putData("CoralGrabberArm", this);
    }

    private void configureMotor() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(true);
        config.smartCurrentLimit(40);
        config.voltageCompensation(12.0);
        config.idleMode(IdleMode.kBrake);

        // Conversion to consider gear ratio
        // One motor rotation = (1/GEAR_RATIO) mechanism rotations
        config.encoder.positionConversionFactor(1.0 / GEAR_RATIO);
        config.encoder.velocityConversionFactor(1.0 / (GEAR_RATIO * 60.0));

        // PID without position wrapping
        config.closedLoop
            .p(0.7)
            .i(0.0)
            .d(0.3)
            .outputRange(-1, 1);

        armMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void setSpeed(double speed) {
        if (Math.abs(speed) > DEADBAND) {
            // Direct speed control when there's input
            armMotor.set(speed);
            wasMoving = true;
        } else {
            if (wasMoving) {
                // Capture position when input stops
                targetPosition = encoder.getPosition();
                wasMoving = false;
            }
            // Hold position
            controller.setReference(targetPosition, ControlType.kPosition);
        }
    }


    
    public void stop() {
        targetPosition = encoder.getPosition();
        controller.setReference(targetPosition, ControlType.kPosition);
    }

    // Método para prueba directa del motor
    public void setDirectOutput(double output) {
        armMotor.set(output);
    }

    // Método para ir a posición fija de prueba
    public void goToTestPosition(double position) {
        targetPosition = position;
        controller.setReference(position, ControlType.kPosition);
    }
    
    @Override
    public void periodic() {
        // SmartDashboard.putNumber("Coral/Position", encoder.getPosition());
        // SmartDashboard.putNumber("Coral/Target", targetPosition);
        // SmartDashboard.putBoolean("Coral/IsHolding", !wasMoving);
    }
    
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("CoralGrabberArm");
        builder.addDoubleProperty("Current Position", encoder::getPosition, null);
        builder.addDoubleProperty("Target Position", () -> targetPosition, null);
    }
}
