package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
// import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SimpleElevatorConstants;

public class CoralGrabberArm extends SubsystemBase {
    // Hardware
    private final SparkMax armMotor;
    private final AbsoluteEncoder encoder;
    private final SparkClosedLoopController controller;

    // Position tracking
    private double targetPosition = 0.0;
    private static final double GEAR_RATIO = 100;
    private static final double MIN_POSITION = -1.0;
    private static final double MAX_POSITION = 1.0;

    public CoralGrabberArm() {
        armMotor = new SparkMax(SimpleElevatorConstants.MOTOR_ID, MotorType.kBrushless);
        encoder = armMotor.getAbsoluteEncoder();
        controller = armMotor.getClosedLoopController();
        
        configureMotor();
    }

    private void configureMotor() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(true);
        config.smartCurrentLimit(40);
        config.voltageCompensation(12.0);
        config.idleMode(IdleMode.kBrake);

        config.encoder.positionConversionFactor(1.0 / GEAR_RATIO);
        config.encoder.velocityConversionFactor(1.0 / (GEAR_RATIO * 60.0));

        config.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .p(0.4)
            .i(0.0)
            .d(0.0)
            .outputRange(-1, 1);

        armMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void setSpeed(double speed) {
        double newTarget = targetPosition + speed * 0.02;
        targetPosition = MathUtil.clamp(newTarget, MIN_POSITION, MAX_POSITION);
        controller.setReference(targetPosition, ControlType.kPosition);
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
