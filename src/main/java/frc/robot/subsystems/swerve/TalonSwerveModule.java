package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.Mk4iModuleConstants;

public class TalonSwerveModule extends SubsystemBase {
    // Drive motor (NEO with SparkMax)
    private final SparkMax drivingMotor;
    private final RelativeEncoder drivingEncoder;
    private final SparkClosedLoopController drivingController;

    // Steering motor (TalonFX with CANcoder)
    private final TalonFX steeringMotor;
    private final CANcoder steerAbsoluteEncoder;

    // Configuration
    private final Mk4iModuleConstants moduleConstants;

    // State variables
    private SwerveModuleState currentState;
    private SwerveModuleState targetState;

    public TalonSwerveModule(Mk4iModuleConstants moduleConstants) {
        this.moduleConstants = moduleConstants;

        // Initialize drive hardware (NEO with SparkMax)
        drivingMotor = new SparkMax(moduleConstants.drivingMotorID(), MotorType.kBrushless);
        drivingEncoder = drivingMotor.getEncoder();
        drivingController = drivingMotor.getClosedLoopController();
        configureDriveMotor();

        // Initialize steering hardware (TalonFX with CANcoder)
        steeringMotor = new TalonFX(moduleConstants.steeringMotorID(), "can_fd");
        steerAbsoluteEncoder = new CANcoder(moduleConstants.steerAbsoluteEncoderID(), "can_fd");
        configureSteeringMotor();
        
        // Initialize states
        currentState = new SwerveModuleState(0, getSteerAngle());
        targetState = new SwerveModuleState(0, currentState.angle);
    }

    private void configureDriveMotor() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(moduleConstants.isDriveMotorInverted());
        config.smartCurrentLimit(Constants.MotorConstants.driveCurrentLimit);
        config.voltageCompensation(Constants.MotorConstants.voltageCompensation);
        config.idleMode(Constants.MotorConstants.driveIdleMode);

        // Set conversion factors
        double drivingFactor = Constants.SwerveConstants.Mk4iMechanicalConstants.drivingFactor;
        config.encoder.positionConversionFactor(drivingFactor);
        config.encoder.velocityConversionFactor(drivingFactor / 60.0);

        // Configure PID
        config.closedLoop.p(moduleConstants.drivingPIDConstants().kP());
        config.closedLoop.i(moduleConstants.drivingPIDConstants().kI());
        config.closedLoop.d(moduleConstants.drivingPIDConstants().kD());
        config.closedLoop.minOutput(moduleConstants.drivingPIDConstants().kMinOutput());
        config.closedLoop.maxOutput(moduleConstants.drivingPIDConstants().kMaxOutput());

        drivingMotor.configure(config);
        drivingEncoder.setPosition(0);
    }

    private void configureSteeringMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        // Basic motor configuration
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = moduleConstants.isSteerMotorInverted() ? 
            InvertedValue.Clockwise_Positive : 
            InvertedValue.CounterClockwise_Positive;

        // Configure CANcoder as feedback device in Fused mode
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        config.Feedback.FeedbackRemoteSensorID = steerAbsoluteEncoder.getDeviceID();
        config.Feedback.RotorToSensorRatio = Constants.SwerveConstants.Mk4iMechanicalConstants.steeringFactor;
        
        // Configure PID
        config.Slot0.kP = moduleConstants.steeringPIDConstants().kP();
        config.Slot0.kI = moduleConstants.steeringPIDConstants().kI();
        config.Slot0.kD = moduleConstants.steeringPIDConstants().kD();
        
        // Enable continuous wrap for steering
        config.ClosedLoopGeneral.ContinuousWrap = true;
        
        // Apply configuration
        steeringMotor.getConfigurator().apply(config);
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        // Update current state
        currentState = new SwerveModuleState(getDriveVelocity(), getSteerAngle());

        // Optimize the reference state to avoid spinning further than 90 degrees
        targetState = SwerveModuleState.optimize(desiredState, currentState.angle);

        // Set motor references
        drivingController.setReference(targetState.speedMetersPerSecond, ControlType.kVelocity);
        steeringMotor.setControl(steeringMotor.getPosition().withPosition(targetState.angle.getRotations()));
    }

    private double getDriveVelocity() {
        return drivingEncoder.getVelocity();
    }

    private Rotation2d getSteerAngle() {
        return Rotation2d.fromRotations(steerAbsoluteEncoder.getAbsolutePosition().getValue());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            drivingEncoder.getPosition(),
            getSteerAngle()
        );
    }

    public SwerveModuleState getCurrentState() {
        return currentState;
    }

    public SwerveModuleState getTargetState() {
        return targetState;
    }
}
