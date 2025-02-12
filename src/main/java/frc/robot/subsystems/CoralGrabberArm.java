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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SimpleElevatorConstants;
import edu.wpi.first.math.MathUtil;

public class CoralGrabberArm extends SubsystemBase {
    // Hardware
    private final SparkMax armMotor;
    private final RelativeEncoder encoder;
    private final SparkClosedLoopController controller;

    // Position tracking
    private double targetPosition = 0.0;
    private static final double MAX_POSITION = 10.0; // Adjust these limits based on mechanism
    private static final double MIN_POSITION = -10.0;
    private static final double GEAR_RATIO = 36; // Verify actual gear ratio
    
    public CoralGrabberArm() {
        armMotor = new SparkMax(SimpleElevatorConstants.MOTOR_ID, MotorType.kBrushless);
        encoder = armMotor.getEncoder();
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
            .p(0.4)
            .i(0.0)
            .d(0.0)
            .outputRange(-1, 1);

        armMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void setSpeed(double speed) {
        double newTarget = targetPosition + speed * 0.01;
        // Limit target position
        targetPosition = MathUtil.clamp(newTarget, MIN_POSITION, MAX_POSITION);
        controller.setReference(targetPosition, ControlType.kPosition);
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
        // Monitor position error
        // double currentPos = encoder.getPosition();
        // double error = targetPosition - currentPos;
    }
    
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("CoralGrabberArm");
        builder.addDoubleProperty("Current Position", encoder::getPosition, null);
        builder.addDoubleProperty("Target Position", () -> targetPosition, null);
    }
}
