package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralConstants;

public class CoralGrabberArm extends SubsystemBase {
    // Hardware
    private final SparkMax armMotor;
    // Absolute encoder
    private final AbsoluteEncoder armEncoder;
    // Closed loop controller
    private final SparkClosedLoopController controller;
    // Gear ratio
    private final double gearRatio = 1.0;
    
    // Predefined positions (in rotations)
    public static final double STOWED_POSITION = 0.0;
    public static final double SCORING_POSITION = Math.PI / 2.0;
    
    public CoralGrabberArm() {
        // Use proper motor ID from CoralConstants
        armMotor = new SparkMax(CoralConstants.ARM_MOTOR_ID, MotorType.kBrushless);
        armEncoder = armMotor.getAbsoluteEncoder();
        controller = armMotor.getClosedLoopController();
        
        SmartDashboard.putData("CoralGrabberArm", this);
        configureMotor();
    }
    
    private void configureMotor() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(true);
        config.smartCurrentLimit(30); // Set appropriate current limit
        config.voltageCompensation(12.0);
        config.idleMode(IdleMode.kBrake);

        double factor = (2.0 * Math.PI) / gearRatio;
        config.absoluteEncoder.positionConversionFactor(factor);
        config.absoluteEncoder.velocityConversionFactor(factor / 60.0);
        
        config.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        config.closedLoop.p(0.015);
        config.closedLoop.i(0.005);
        config.closedLoop.d(0.01);
        config.closedLoop.minOutput(-1);
        config.closedLoop.maxOutput(1);
        config.closedLoop.positionWrappingEnabled(true);
        config.closedLoop.positionWrappingInputRange(-Math.PI,Math.PI);
    
        armMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setArmPosition(double position) {
        controller.setReference(position, ControlType.kPosition);
    }

    public double getArmPosition() {
        return armEncoder.getPosition();
    }
    
    // Simple joystick control methods
    public void setSpeed(double speed) {
        armMotor.set(speed);
    }
    
    public void stop() {
        armMotor.set(0);
    }
    
    @Override
    public void periodic() {

    }
    
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("CoralGrabberArm");
        builder.addDoubleProperty("Position", this::getArmPosition, null);
    }
}
