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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
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
    public static final Rotation2d STOWED_POSITION = Rotation2d.fromDegrees(0);
    public static final Rotation2d SCORING_POSITION = Rotation2d.fromDegrees(90);
    
    public CoralGrabberArm() {
        // Use proper motor ID from CoralConstants
        armMotor = new SparkMax(CoralConstants.ARM_MOTOR_ID, MotorType.kBrushless);
        armEncoder = armMotor.getAbsoluteEncoder();
        controller = armMotor.getClosedLoopController();
        
        SmartDashboard.putData("CoralGrabberArm", this);
        configureMotor();
    }
    
    private void configureMotor() {
        double factor = (2.0 * Math.PI) / gearRatio;

        var turnConfig = new SparkMaxConfig();
        turnConfig
            .inverted(true)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(20)
            .voltageCompensation(12.0);
        turnConfig
            .absoluteEncoder
            .inverted(false)
            .positionConversionFactor(factor)
            .velocityConversionFactor(factor / 60.0)
            .averageDepth(2);
        turnConfig
            .closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(0, 2 * Math.PI)
            .pidf(1, 0.0, 0, 0.0);
        turnConfig
            .signals
            .absoluteEncoderPositionAlwaysOn(true)
            .absoluteEncoderPositionPeriodMs((int) (1000.0 / 100))
            .absoluteEncoderVelocityAlwaysOn(true)
            .absoluteEncoderVelocityPeriodMs(20)
            .appliedOutputPeriodMs(20)
            .busVoltagePeriodMs(20)
            .outputCurrentPeriodMs(20);
    
        armMotor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setArmPosition(Rotation2d rotation) {
        double setpoint =
            MathUtil.inputModulus(
                rotation.plus(new Rotation2d()).getRadians(), 0, 2 * Math.PI);
        controller.setReference(setpoint, ControlType.kPosition);
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
